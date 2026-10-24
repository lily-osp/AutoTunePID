// Comparative benchmark harness for the AutoTunePID library (lily-osp/AutoTunePID v1.2.0).
// Compiles the unmodified library source on the host and drives it through:
//   1. Relay-feedback auto-tuning against simulated FOPDT/SOPDT plants.
//   2. Closed-loop setpoint-step evaluation (clean and multi-seed/noise).
//   3. Lambda sensitivity study for the model-based methods (IMC, Lambda).
//   4. Resource cost (ns per deterministic update, sizeof).
//   5. Relay identification audit: independent re-estimation from raw traces
//      plus a bias-centered relay ablation (harness-side relay).
//   6. Track B tuning-rule validation with ground-truth plant parameters,
//      including a canonical reaction-curve Cohen-Coon reference.
//   7. Anti-windup mechanism audit: analytic offset prediction vs observed,
//      with an anti-windup-disabled ablation.
//   8. Plant-gain sensitivity sweep (K = 0.5, 2) on P2 dynamics.
// Outputs: results/*.csv, results/*.tex (table fragments), figures/*.csv.
#include "Arduino.h"
#include "AutoTunePID.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iomanip>
#include <random>
#include <sstream>
#include <string>
#include <vector>

uint32_t g_simMillis = 0U;

namespace {

constexpr double kDt = 0.1;     // simulation sample time [s], matches library 100 ms tick
constexpr double kSp = 100.0;   // setpoint in engineering units
constexpr double kTuneCap = 7200.0;  // hard cap on a single tuning experiment [s]
constexpr double kSimHorizon = 1800.0; // closed-loop evaluation horizon [s]
constexpr double kImax = 0.8 * 255.0;  // library anti-windup clamp at defaults

struct PlantSpec {
    std::string id;
    std::string klass;
    double K;
    double T;
    double L;
    double T2; // > 0 turns the model into SOPDT (two lags + delay)
};

const std::vector<PlantSpec> kPlants = {
    {"P1", "FOPDT lag-dominant (L/T=0.10)", 1.0, 20.0, 2.0, 0.0},
    {"P2", "FOPDT moderate (L/T=0.50)",     1.0, 10.0, 5.0, 0.0},
    {"P3", "FOPDT delay-dominant (L/T=2.0)",1.0,  4.0, 8.0, 0.0},
    {"P4", "FOPDT slow thermal (T=60 s)",   1.0, 60.0, 6.0, 0.0},
    {"P5", "SOPDT (T1=15, T2=5, L=3)",      1.0, 15.0, 3.0, 5.0},
};

class Plant {
public:
    explicit Plant(const PlantSpec& s)
        : s_(s)
        , k1_(1.0 - std::exp(-kDt / s.T))
        , k2_(s.T2 > 0.0 ? 1.0 - std::exp(-kDt / s.T2) : 0.0)
        , buf_(static_cast<size_t>(std::max(1.0, std::round(s.L / kDt))) + 1U, 0.0)
    {
    }

    double step(double u)
    {
        buf_.push_back(u);
        const double ue = s_.K * buf_.front();
        buf_.pop_front();
        if (s_.T2 > 0.0) {
            y2_ += (ue - y2_) * k1_;
            y_ += (y2_ - y_) * k2_;
        } else {
            y_ += (ue - y_) * k1_;
        }
        return y_;
    }

    double y() const { return y_; }
    void reset()
    {
        y_ = 0.0;
        y2_ = 0.0;
        std::fill(buf_.begin(), buf_.end(), 0.0);
    }

private:
    PlantSpec s_;
    double k1_;
    double k2_;
    double y_ = 0.0;
    double y2_ = 0.0;
    std::deque<double> buf_;
};

double phaseAt(const PlantSpec& s, double w)
{
    if (s.T2 > 0.0) {
        return w * s.L + std::atan(w * s.T) + std::atan(w * s.T2);
    }
    return w * s.L + std::atan(w * s.T);
}

void criticalPoint(const PlantSpec& s, double& wc, double& ku, double& tu)
{
    double lo = 1e-4;
    double hi = 50.0;
    for (int i = 0; i < 200; ++i) {
        const double mid = 0.5 * (lo + hi);
        if (phaseAt(s, mid) < M_PI) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    wc = 0.5 * (lo + hi);
    const double magInv = (s.T2 > 0.0)
        ? std::sqrt(1.0 + std::pow(wc * s.T, 2)) * std::sqrt(1.0 + std::pow(wc * s.T2, 2))
        : std::sqrt(1.0 + std::pow(wc * s.T, 2));
    ku = magInv / s.K;
    tu = 2.0 * M_PI / wc;
}

struct TuneResult {
    bool ok = false;
    double tuneTime = -1.0;
    float ku = 0.0f;
    float tu = 0.0f;
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
};

struct Metrics {
    bool reachedBand = false;
    double overshootPct = -1.0;
    double riseTime = -1.0;
    double settleTime = -1.0;
    bool settled = false;    // 2% band, dwell = final 5% of horizon
    bool settled5 = false;   // 5% band
    bool settled10 = false;  // 10% band
    double iae = 0.0;
    double ise = 0.0;
    double essAbs = -1.0;      // mean |e| over final tenth of horizon
    double essTailStd = -1.0;  // std of signed e over the same window
};

std::string methodTag(atp::TuningMethod m)
{
    switch (m) {
    case atp::TuningMethod::ZieglerNichols: return "ZN";
    case atp::TuningMethod::CohenCoon:      return "CC";
    case atp::TuningMethod::IMC:            return "IMC";
    case atp::TuningMethod::TyreusLuyben:   return "TL";
    case atp::TuningMethod::LambdaTuning:   return "Lambda";
    default:                                return "Manual";
    }
}

std::string oscTag(atp::OscillationMode m)
{
    switch (m) {
    case atp::OscillationMode::Normal: return "Normal";
    case atp::OscillationMode::Half:   return "Half";
    case atp::OscillationMode::Mild:   return "Mild";
    default:                           return "?";
    }
}

TuneResult runTune(const PlantSpec& ps, atp::TuningMethod method, atp::OscillationMode om,
                   double lambdaOverride, bool useLambda)
{
    Plant plant(ps);
    atp::AutoTunePID pid(0.0f, 255.0f, method);
    pid.setSetpoint(static_cast<float>(kSp));
    pid.setOscillationMode(om);
    if (useLambda) {
        pid.setLambda(static_cast<float>(lambdaOverride));
    }
    pid.setOperationalMode(atp::OperationalMode::Tune);

    TuneResult r;
    double t = 0.0;
    while (t < kTuneCap) {
        g_simMillis += 100U;
        plant.step(pid.getOutput());
        pid.update(static_cast<float>(plant.y()), static_cast<float>(kDt));
        t += kDt;
        if (pid.getOperationalMode() != atp::OperationalMode::Tune) {
            r.ok = true;
            break;
        }
    }
    r.tuneTime = t;
    r.ku = pid.getKu();
    r.tu = pid.getTu();
    r.kp = pid.getKp();
    r.ki = pid.getKi();
    r.kd = pid.getKd();
    return r;
}

// Reference implementation of the recommended anti-windup repair: identical to
// the library's parallel law with derivative-on-measurement, but the integral
// CONTRIBUTION Ki*I is clamped to the output span instead of the raw
// accumulator. Used only as an ablation arm; it is not shipped library code.
struct HostRepairPid {
    float kp, ki, kd;
    float integ = 0.0f;
    float prevIn = 0.0f;
    float out = 0.0f;
    float step(float sp, float y, float dt)
    {
        const float e = sp - y;
        const float deriv = -(y - prevIn) / dt;
        prevIn = y;
        integ += e * dt;
        const float span = 255.0f;
        if (integ * ki > span) integ = span / ki;
        else if (integ * ki < -span) integ = -span / ki;
        float u = kp * e + ki * integ + kd * deriv;
        if (u < 0.0f) u = 0.0f;
        if (u > 255.0f) u = 255.0f;
        out = u;
        return u;
    }
};

Metrics runEval(const PlantSpec& ps, const TuneResult& tune, bool noise, unsigned seed,
                const char* figPath, bool antiWindup = true, double noiseAmpPct = 1.0,
                bool useRepair = false)
{
    Plant plant(ps);
    atp::AutoTunePID pid(0.0f, 255.0f, atp::TuningMethod::Manual);
    pid.setManualGains(tune.kp, tune.ki, tune.kd);
    pid.setSetpoint(static_cast<float>(kSp));
    pid.enableAntiWindup(antiWindup, 0.8f);
    pid.setOperationalMode(atp::OperationalMode::Normal);
    HostRepairPid repair{tune.kp, tune.ki, tune.kd};

    std::mt19937 gen(seed);
    std::normal_distribution<double> nd(0.0, noiseAmpPct / 100.0 * kSp);

    std::ofstream fig;
    if (figPath != nullptr) {
        fig.open(figPath);
        fig << "t y\n";
    }

    const long n = static_cast<long>(kSimHorizon / kDt);
    double maxY = 0.0;
    double t10 = -1.0;
    double t90 = -1.0;
    double lastV2 = -1.0;   // last time outside the 2% band
    double lastV5 = -1.0;   // 5% band
    double lastV10 = -1.0;  // 10% band
    double iae = 0.0;
    double ise = 0.0;
    double essAccum = 0.0;
    double essSumAccum = 0.0;
    double essSqAccum = 0.0;
    long essCount = 0;
    const double b2 = 0.02 * kSp;
    const double b5 = 0.05 * kSp;
    const double b10 = 0.10 * kSp;

    for (long i = 1; i <= n; ++i) {
        g_simMillis += 100U;
        const double meas = plant.y() + (noise ? nd(gen) : 0.0);
        if (useRepair) {
            plant.step(repair.step(static_cast<float>(kSp), static_cast<float>(meas),
                                   static_cast<float>(kDt)));
        } else {
            pid.update(static_cast<float>(meas), static_cast<float>(kDt));
            plant.step(pid.getOutput());
        }

        const double t = i * kDt;
        const double y = plant.y();
        const double e = kSp - y;
        iae += std::fabs(e) * kDt;
        ise += e * e * kDt;
        maxY = std::max(maxY, y);
        if (y >= 0.1 * kSp && t10 < 0.0) t10 = t;
        if (y >= 0.9 * kSp && t90 < 0.0) t90 = t;
        const double ae = std::fabs(e);
        if (ae > b2) lastV2 = t;
        if (ae > b5) lastV5 = t;
        if (ae > b10) lastV10 = t;
        if (i > n - n / 10) {
            essAccum += ae;
            essSumAccum += e;
            essSqAccum += e * e;
            ++essCount;
        }
        if (fig.is_open() && (i % 20 == 0)) {
            fig << std::fixed << std::setprecision(1) << t << " " << std::setprecision(3) << y << "\n";
        }
    }

    Metrics m;
    m.reachedBand = (t90 > 0.0);
    m.overshootPct = std::max(0.0, (maxY - kSp) / kSp * 100.0);
    m.riseTime = (t10 > 0.0 && t90 > 0.0) ? t90 - t10 : -1.0;
    m.settleTime = lastV2;
    m.settled = (lastV2 >= 0.0 && lastV2 < 0.95 * kSimHorizon && m.reachedBand);
    m.settled5 = (lastV5 >= 0.0 && lastV5 < 0.95 * kSimHorizon && m.reachedBand);
    m.settled10 = (lastV10 >= 0.0 && lastV10 < 0.95 * kSimHorizon && m.reachedBand);
    m.iae = iae;
    m.ise = ise;
    m.essAbs = essCount > 0 ? essAccum / essCount : -1.0;
    if (essCount > 0) {
        const double mu = essSumAccum / essCount;
        m.essTailStd = std::sqrt(std::max(0.0, essSqAccum / essCount - mu * mu));
    }
    return m;
}

// ---- Relay identification audit ---------------------------------------------

struct RelayTrace {
    std::vector<double> t;
    std::vector<double> y;
    std::vector<double> u;
};

RelayTrace recordTuneTrace(const PlantSpec& ps, atp::OscillationMode om)
{
    Plant plant(ps);
    atp::AutoTunePID pid(0.0f, 255.0f, atp::TuningMethod::ZieglerNichols);
    pid.setSetpoint(static_cast<float>(kSp));
    pid.setOscillationMode(om);
    pid.setOperationalMode(atp::OperationalMode::Tune);

    RelayTrace tr;
    double t = 0.0;
    while (t < kTuneCap) {
        g_simMillis += 100U;
        plant.step(pid.getOutput());
        pid.update(static_cast<float>(plant.y()), static_cast<float>(kDt));
        tr.t.push_back(t);
        tr.y.push_back(plant.y());
        tr.u.push_back(pid.getOutput());
        t += kDt;
        if (pid.getOperationalMode() != atp::OperationalMode::Tune) {
            break;
        }
    }
    return tr;
}

struct IndepEstimate {
    bool ok = false;
    double ku = -1.0;
    double tu = -1.0;
    double relayD = -1.0;
    double dutyHigh = -1.0; // fraction of time relay spends at the high level
};

// Independent Ku/Tu extraction: same-direction setpoint crossings for the
// period, peak-to-peak amplitude over the steady half of the trace, relay
// amplitude and duty cycle from the logged output. Shares nothing with the
// library's internal bookkeeping.
IndepEstimate independentEstimate(const RelayTrace& tr)
{
    IndepEstimate est;
    if (tr.t.size() < 10) {
        return est;
    }
    double umax = -1e18;
    double umin = 1e18;
    double usum = 0.0;
    for (double v : tr.u) {
        umax = std::max(umax, v);
        umin = std::min(umin, v);
        usum += v;
    }
    est.relayD = (umax - umin) / 2.0;
    est.dutyHigh = usum / static_cast<double>(tr.u.size());

    std::vector<double> up;
    for (size_t i = 1; i < tr.y.size(); ++i) {
        if (tr.y[i - 1] < kSp && tr.y[i] >= kSp) {
            up.push_back(tr.t[i]);
        }
    }
    if (up.size() < 3) {
        return est;
    }
    // steady cycles: second half of the crossings
    const size_t half = up.size() / 2;
    const double t0 = up[half - 1];
    est.tu = (up.back() - t0) / static_cast<double>(up.size() - half);

    double ymax = -1e18;
    double ymin = 1e18;
    for (size_t i = 0; i < tr.y.size(); ++i) {
        if (tr.t[i] >= t0) {
            ymax = std::max(ymax, tr.y[i]);
            ymin = std::min(ymin, tr.y[i]);
        }
    }
    const double amp = (ymax - ymin) / 2.0;
    if (amp > 1e-3) {
        est.ku = 4.0 * est.relayD / (M_PI * amp);
        est.ok = true;
    }
    return est;
}

// Ablation: harness-side relay centered on the actuator bias point
// u_eq = SP/K with the same amplitude as the library's Normal mode, the same
// crossing logic, and the same describing-function post-processing. Also fills
// a trace so the independent steady-state estimator can be applied to it,
// completing the centering x estimator factorial.
TuneResult biasCenteredRelay(const PlantSpec& ps, int halfCycles, RelayTrace* traceOut)
{
    Plant plant(ps);
    const double d = 127.5;             // (255 - 0) / 2
    const double ueq = kSp / ps.K;      // actuator bias at the setpoint
    bool high = true;
    int crossings = 0;
    double lastCross = -1.0;
    double period = -1.0;
    double ampSum = 0.0;
    int ampN = 0;
    double ymax = -1e18;
    double ymin = 1e18;
    double t = 0.0;

    while (t < kTuneCap) {
        g_simMillis += 100U;
        const double u = high ? ueq + d : ueq - d;
        plant.step(u);
        const double y = plant.y();
        if (traceOut != nullptr) {
            traceOut->t.push_back(t);
            traceOut->y.push_back(y);
            traceOut->u.push_back(u);
        }
        ymax = std::max(ymax, y);
        ymin = std::min(ymin, y);
        bool crossed = false;
        if (high && y > kSp) {
            high = false;
            crossed = true;
        } else if (!high && y < kSp) {
            high = true;
            crossed = true;
        }
        if (crossed) {
            if (crossings > 0) {
                period = 2.0 * (t - lastCross);
                const double a = (ymax - ymin) / 2.0;
                if (a > 1e-3) {
                    ampSum += a;
                    ++ampN;
                }
            }
            lastCross = t;
            ++crossings;
            ymax = -1e18;
            ymin = 1e18;
        }
        t += kDt;
        if (crossings >= halfCycles) {
            break;
        }
    }

    TuneResult r;
    r.ok = (ampN > 0 && period > 0.0);
    r.tuneTime = t;
    if (r.ok) {
        r.ku = static_cast<float>(4.0 * d / (M_PI * (ampSum / ampN)));
        r.tu = static_cast<float>(period);
    }
    return r;
}

// ---- Track B: tuning rules fed ground-truth parameters ----------------------

// Effective FOPDT parameters (Skogestad half-rule for SOPDT).
void effectiveFopdt(const PlantSpec& s, double& T, double& L)
{
    T = s.T;
    L = s.L + (s.T2 > 0.0 ? 0.5 * s.T2 : 0.0);
}

// Canonical reaction-curve Cohen-Coon PID (Cohen & Coon, 1953), parallel form.
void canonicalCohenCoon(const PlantSpec& s, float& kp, float& ki, float& kd)
{
    double T = 0.0, L = 0.0;
    effectiveFopdt(s, T, L);
    const double b = T / L;
    const double kpc = (1.0 / s.K) * (4.0 / 3.0 * b + 0.25);
    const double ti = L * (32.0 + 6.0 * b) / (13.0 + 8.0 * b);
    const double td = 4.0 * L / (11.0 + 2.0 * b);
    kp = static_cast<float>(kpc);
    ki = static_cast<float>(kpc / ti);
    kd = static_cast<float>(kpc * td);
}

struct TrackBVariant {
    std::string tag;
    int kind; // 0 ZN(true), 1 CC* canonical, 2 TL(true), 3 IMC(true, lam), 4 IMC(true, T)
    double lam = 0.5;
};

TuneResult trackBGains(const PlantSpec& ps, const TrackBVariant& v)
{
    double wc = 0.0, kuT = 0.0, tuT = 0.0;
    criticalPoint(ps, wc, kuT, tuT);
    double T = 0.0, L = 0.0;
    effectiveFopdt(ps, T, L);

    TuneResult r;
    r.ok = true;
    r.ku = static_cast<float>(kuT);
    r.tu = static_cast<float>(tuT);
    switch (v.kind) {
    case 0:
        r.kp = static_cast<float>(0.6 * kuT);
        r.ki = static_cast<float>(2.0 * 0.6 * kuT / tuT);
        r.kd = static_cast<float>(0.125 * 0.6 * kuT * tuT);
        break;
    case 1:
        canonicalCohenCoon(ps, r.kp, r.ki, r.kd);
        break;
    case 2:
        r.kp = static_cast<float>(kuT / 2.2);
        r.ki = static_cast<float>(kuT / 2.2 / (2.2 * tuT));
        r.kd = static_cast<float>(kuT / 2.2 * tuT / 6.3);
        break;
    default: {
        const double lam = (v.kind == 4) ? T : v.lam;
        const double kp = T / (ps.K * (lam + L));
        r.kp = static_cast<float>(kp);
        r.ki = static_cast<float>(kp / T);
        r.kd = static_cast<float>(kp * L / 2.0);
        break;
    }
    }
    return r;
}

double benchUpdateNs()
{
    atp::AutoTunePID pid(0.0f, 255.0f, atp::TuningMethod::ZieglerNichols);
    pid.setManualGains(2.0f, 0.1f, 0.05f);
    pid.setSetpoint(100.0f);
    pid.setOperationalMode(atp::OperationalMode::Normal);
    const int n = 300000;
    double sink = 0.0;
    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < n; ++i) {
        pid.update(92.5f, 0.1f);
        sink += pid.getOutput();
    }
    const auto t1 = std::chrono::steady_clock::now();
    if (sink == 12345.678) {
        std::printf(" "); // consume sink so the loop cannot be optimized away
    }
    return std::chrono::duration<double, std::nano>(t1 - t0).count() / n;
}

std::string fmtS(double v)
{
    if (v < 0.0) return "--"; // -1 sentinel: metric not reached
    std::ostringstream os;
    os << std::fixed << std::setprecision(1) << v;
    return os.str();
}

// Signed formatter for error percentages; only the -999 sentinel is missing.
std::string fmtSign(double v)
{
    if (v <= -900.0) return "--";
    std::ostringstream os;
    os << (v >= 0.0 ? "+" : "-") << std::fixed << std::setprecision(1) << std::fabs(v);
    return os.str();
}

std::string fmtG(double v)
{
    std::ostringstream os;
    os << std::fixed << std::setprecision(3) << v;
    return os.str();
}

double mean(const std::vector<double>& v)
{
    double s = 0.0;
    for (double x : v) s += x;
    return v.empty() ? 0.0 : s / v.size();
}

double stdev(const std::vector<double>& v)
{
    if (v.size() < 2) return 0.0;
    const double m = mean(v);
    double s = 0.0;
    for (double x : v) s += (x - m) * (x - m);
    return std::sqrt(s / (v.size() - 1));
}

} // namespace

int main()
{
    const char* outDir = "results";
    const char* figDir = "figures";

    std::ofstream metricsCsv(std::string(outDir) + "/metrics.csv");
    std::ofstream estCsv(std::string(outDir) + "/estimation.csv");
    std::ofstream iaeCsv(std::string(figDir) + "/iae.csv");
    std::ofstream noiseCsv(std::string(outDir) + "/noise_stats.csv");
    std::ofstream relayCsv(std::string(outDir) + "/relay_audit.csv");
    std::ofstream trackBCsv(std::string(outDir) + "/trackb.csv");
    std::ofstream awCsv(std::string(outDir) + "/aw_ablation.csv");
    std::ofstream gainCsv(std::string(outDir) + "/gain_sweep.csv");
    std::ofstream verCsv(std::string(outDir) + "/verification.csv");
    // Table fragments carry complete tabular environments; \input stays outside
    // tabular, otherwise the fragment's trailing \\ breaks the following \bottomrule.
    std::ofstream tableMain(std::string(outDir) + "/table_main.tex");
    std::ofstream tableEst(std::string(outDir) + "/table_est.tex");
    std::ofstream tableLambda(std::string(outDir) + "/table_lambda.tex");
    std::ofstream tablePhaseB(std::string(outDir) + "/table_phaseb.tex");
    std::ofstream tableRelay(std::string(outDir) + "/table_relay.tex");
    std::ofstream tableTrackB(std::string(outDir) + "/table_trackb.tex");
    std::ofstream tableAW(std::string(outDir) + "/table_aw.tex");
    std::ofstream tableGain(std::string(outDir) + "/table_gain.tex");
    std::ofstream tableClass(std::string(outDir) + "/table_class.tex");
    std::ofstream tableBand(std::string(outDir) + "/table_band.tex");
    std::ofstream summary(std::string(outDir) + "/summary.txt");

    // Fail fast if the output directories are missing: ofstream does not throw
    // by default and would otherwise swallow every table write.
    if (!metricsCsv || !summary || !tableMain || !tableEst || !tableLambda
        || !tablePhaseB || !tableRelay || !tableTrackB || !tableAW || !tableGain) {
        std::fprintf(stderr, "error: cannot open files in %s/ or %s/; mkdir them first\n",
                     outDir, figDir);
        return 1;
    }

    tableEst << "\\begin{tabular}{@{}llcccccc@{}}\n\\toprule\n"
                "Plant & Mode & $K_u$ & $T_u$ & $K_u^{\\mathit{true}}$ & $T_u^{\\mathit{true}}$ & $K_u$ err & $T_u$ err \\\\\n"
                " & & est. & est.\\,[s] & & [s] & [\\%] & [\\%] \\\\\n\\midrule\n";
    tableMain << "\\begin{tabular}{@{}llccccccccc@{}}\n\\toprule\n"
                 "Plant & Method & $K_p$ & $K_i$ & $K_d$ & OS\\,[\\%] & $t_r$\\,[s] & $t_s$\\,[s] & IAE & IAE$_{1\\%}$ & $|e|_\\infty$ \\\\\n"
                 "\\midrule\n";
    tableLambda << "\\begin{tabular}{@{}llcccccc@{}}\n\\toprule\n"
                   "Method & $\\lambda$\\,[s] & $K_p$ & $K_i$ & $K_d$ & OS\\,[\\%] & $t_r$\\,[s] & IAE \\\\\n"
                   "\\midrule\n";
    tablePhaseB << "\\begin{tabular}{@{}lcccccc@{}}\n\\toprule\n"
                   " & \\multicolumn{2}{c}{$K_p$} & \\multicolumn{2}{c}{$K_i$} & \\multicolumn{2}{c}{$K_d$} \\\\\n"
                   "Method & library & reference & library & reference & library & reference \\\\\n"
                   "\\midrule\n";
    tableRelay << "\\begin{tabular}{@{}lcccccc@{}}\n\\toprule\n"
                  "Plant & $K_u$ & $K_u$ & $K_u$ & $K_u$ & $T_u$ & $T_u$ \\\\\n"
                  " & lib. & steady & bias & bias+steady & lib. & bias \\\\\n"
                  "\\midrule\n";
    tableTrackB << "\\begin{tabular}{@{}llccccccccc@{}}\n\\toprule\n"
                   "Plant & Rule & $K_p$ & $K_i$ & $K_d$ & OS\\,[\\%] & $t_r$\\,[s] & $t_s$\\,[s] & IAE & $|e|_\\infty$ & $|e|_\\infty$ \\\\\n"
                   " & & & & & & & & & AW off & AW on \\\\\n"
                   "\\midrule\n";
    tableAW << "\\begin{tabular}{@{}llcccccc@{}}\n\\toprule\n"
               "Plant & Rule & $K_i$ & $I_{eq}$ & \\multicolumn{4}{c}{$|e|_\\infty$ [units]} \\\\\n"
               " & & & req. & pred. & AW on & AW off & repair \\\\\n"
               "\\midrule\n";
    tableGain << "\\begin{tabular}{@{}lcccc@{}}\n\\toprule\n"
                 " & \\multicolumn{2}{c}{$K=0.5$} & \\multicolumn{2}{c}{$K=2.0$} \\\\\n"
                 "Method & OS\\,[\\%] & $|e|_\\infty$ & OS\\,[\\%] & $|e|_\\infty$ \\\\\n"
                 "\\midrule\n";
    tableClass << "\\begin{tabular}{@{}lllcl@{}}\n\\toprule\n"
                  "Plant & Method & Clean class & Success\\,[\\%] & Noise class \\\\\n"
                  " & & & at 1\\% noise & (dominant) \\\\\n"
                  "\\midrule\n";
    tableBand << "\\begin{tabular}{@{}llccc@{}}\n\\toprule\n"
                 "Band & Condition & \\multicolumn{3}{c}{Settled fraction [\\%]} \\\\\n"
                 " & & mean & min & max \\\\\n"
                 "\\midrule\n";

    metricsCsv << "plant,method,osc_mode,lambda,ok,tune_s,Ku_est,Tu_est,Ku_true,Tu_true,"
                  "Kp,Ki,Kd,overshoot_pct,rise_s,settle_s,settled,IAE,ISE,ess_abs,e_pred,"
                  "IAE_noise1_mean,IAE_noise1_sd,class_clean,success_pct_1pct,settled5,"
                  "settled10,ess_tail_std\n";
    estCsv << "plant,osc_mode,Ku_est,Tu_est,Ku_true,Tu_true,Ku_err_pct,Tu_err_pct,tune_s\n";
    iaeCsv << "plant,method,iae_clean,iae_noise\n";
    noiseCsv << "plant,method,amp_pct,iae_mean,iae_sd,settled2_frac,settled5_frac,settled10_frac\n";
    relayCsv << "plant,Ku_true,Tu_true,ku_lib,ku_lib_err,ku_ind,ku_ind_err,ku_bias,ku_bias_err,"
                "ku_bias_steady,ku_bias_steady_err,tu_lib,tu_lib_err,tu_bias,tu_bias_err,duty_high\n";
    trackBCsv << "plant,rule,Kp,Ki,Kd,overshoot_pct,rise_s,settle_s,settled,IAE,ess_abs,ess_abs_aw_on\n";
    awCsv << "plant,rule,Ki,Ieq,e_pred,e_obs_aw_on,e_obs_aw_off,e_obs_repaired\n";
    gainCsv << "method,plant_gain,overshoot_pct,ess_abs,iae\n";
    verCsv << "method,Ku_used,Tu_used,Kp_lib,Ki_lib,Kd_lib,Kp_ref,Ki_ref,Kd_ref,max_rel_diff_ppm\n";

    const std::vector<atp::TuningMethod> methods = {
        atp::TuningMethod::ZieglerNichols, atp::TuningMethod::CohenCoon,
        atp::TuningMethod::IMC, atp::TuningMethod::TyreusLuyben,
        atp::TuningMethod::LambdaTuning};

    // ---- Section 1: relay estimation accuracy per oscillation mode ----
    summary << "== Relay estimation accuracy (method independent) ==\n";
    for (const auto& ps : kPlants) {
        double wc = 0.0, kuTrue = 0.0, tuTrue = 0.0;
        criticalPoint(ps, wc, kuTrue, tuTrue);
        for (auto om : {atp::OscillationMode::Normal, atp::OscillationMode::Half,
                        atp::OscillationMode::Mild}) {
            const TuneResult tr = runTune(ps, atp::TuningMethod::ZieglerNichols, om, 0.5, false);
            const double kuErr = tr.ok ? (tr.ku - kuTrue) / kuTrue * 100.0 : 0.0;
            const double tuErr = tr.ok ? (tr.tu - tuTrue) / tuTrue * 100.0 : 0.0;
            estCsv << ps.id << "," << oscTag(om) << "," << tr.ku << "," << tr.tu << ","
                   << kuTrue << "," << tuTrue << "," << kuErr << "," << tuErr << ","
                   << tr.tuneTime << "\n";
            tableEst << ps.id << " & " << oscTag(om) << " & "
                     << fmtG(tr.ku) << " & " << fmtS(tr.tu) << " & "
                     << fmtG(kuTrue) << " & " << fmtS(tuTrue) << " & "
                     << fmtSign(kuErr) << "\\% & " << fmtSign(tuErr) << "\\% \\\\\n";
            summary << ps.id << " " << oscTag(om)
                    << ": Ku " << tr.ku << " (true " << kuTrue << ", err " << kuErr << "%)"
                    << " Tu " << tr.tu << " (true " << tuTrue << ", err " << tuErr << "%)"
                    << " tune " << tr.tuneTime << "s ok=" << tr.ok << "\n";
        }
    }
    tableEst << "\\bottomrule\n\\end{tabular}\n";

    // ---- Section 1b: Phase B gain-rule verification ----
    summary << "\n== Phase B: gain-rule verification on P2 (Normal mode) ==\n";
    {
        const PlantSpec* pB = &kPlants[1];
        for (auto m : methods) {
            const TuneResult tr = runTune(*pB, m, atp::OscillationMode::Normal, 0.5, false);
            const double ku = tr.ku;
            const double tu = tr.tu;
            double kpR = 0.0, kiR = 0.0, kdR = 0.0;
            switch (m) {
            case atp::TuningMethod::ZieglerNichols:
                kpR = 0.6 * ku; kiR = 2.0 * kpR / tu; kdR = 0.125 * kpR * tu; break;
            case atp::TuningMethod::CohenCoon:
                kpR = 0.8 * ku; kiR = kpR / (0.8 * tu); kdR = 0.194 * kpR * tu; break;
            case atp::TuningMethod::TyreusLuyben:
                kpR = ku / 2.2; kiR = kpR / (2.2 * tu); kdR = kpR * tu / 6.3; break;
            default: {
                const double T = 0.67 * tu;
                const double L = 0.17 * tu;
                kpR = T / (0.5 + L);
                kiR = kpR / T;
                kdR = 0.5 * kpR * L;
                break;
            }
            }
            const double dKp = std::fabs((tr.kp - kpR) / kpR) * 1e6;
            const double dKi = std::fabs((tr.ki - kiR) / kiR) * 1e6;
            const double dKd = std::fabs((tr.kd - kdR) / kdR) * 1e6;
            const double dMax = std::max(dKp, std::max(dKi, dKd));
            verCsv << methodTag(m) << "," << ku << "," << tu << ","
                   << tr.kp << "," << tr.ki << "," << tr.kd << ","
                   << kpR << "," << kiR << "," << kdR << "," << dMax << "\n";
            tablePhaseB << methodTag(m) << " & "
                        << fmtG(tr.kp) << " & " << fmtG(kpR) << " & "
                        << fmtG(tr.ki) << " & " << fmtG(kiR) << " & "
                        << fmtG(tr.kd) << " & " << fmtG(kdR) << " \\\\\n";
            summary << methodTag(m) << " max_rel_diff=" << dMax << " ppm"
                    << " (Ku=" << ku << " Tu=" << tu << ")\n";
        }
    }
    tablePhaseB << "\\bottomrule\n\\end{tabular}\n";

    // ---- Section 1c: relay identification audit + bias-centered ablation ----
    summary << "\n== Relay audit: library vs independent estimate vs bias-centered ablation ==\n";
    {
        std::vector<double> vLib, vSteady, vBias, vBS, vTuLib, vTuBias;
        for (const auto& ps : kPlants) {
        double wc = 0.0, kuTrue = 0.0, tuTrue = 0.0;
        criticalPoint(ps, wc, kuTrue, tuTrue);

        const TuneResult lib = runTune(ps, atp::TuningMethod::ZieglerNichols,
                                       atp::OscillationMode::Normal, 0.5, false);
        const RelayTrace trace = recordTuneTrace(ps, atp::OscillationMode::Normal);
        const IndepEstimate ind = independentEstimate(trace);
        RelayTrace biasTrace;
        const TuneResult bias = biasCenteredRelay(ps, 10, &biasTrace);
        const IndepEstimate indBias = independentEstimate(biasTrace);

        // raw trace export for independent verification of the audit
        {
            std::ofstream trOut(std::string(outDir) + "/relay_trace_" + ps.id + ".csv");
            trOut << "t,y,u\n";
            for (size_t i = 0; i < trace.t.size(); ++i) {
                trOut << std::fixed << std::setprecision(1) << trace.t[i] << ","
                      << std::setprecision(4) << trace.y[i] << "," << trace.u[i] << "\n";
            }
        }

        const double kuLibErr = lib.ok ? (lib.ku - kuTrue) / kuTrue * 100.0 : -999.0;
        const double tuLibErr = lib.ok ? (lib.tu - tuTrue) / tuTrue * 100.0 : -999.0;
        const double kuIndErr = ind.ok ? (ind.ku - kuTrue) / kuTrue * 100.0 : -999.0;
        const double kuBiasErr = bias.ok ? (bias.ku - kuTrue) / kuTrue * 100.0 : -999.0;
        const double tuBiasErr = bias.ok ? (bias.tu - tuTrue) / tuTrue * 100.0 : -999.0;
        const double kuBiasSteadyErr = indBias.ok ? (indBias.ku - kuTrue) / kuTrue * 100.0 : -999.0;

        vLib.push_back(kuLibErr); vSteady.push_back(kuIndErr);
        vBias.push_back(kuBiasErr); vBS.push_back(kuBiasSteadyErr);
        vTuLib.push_back(tuLibErr); vTuBias.push_back(tuBiasErr);

        relayCsv << ps.id << "," << kuTrue << "," << tuTrue << ","
                 << lib.ku << "," << kuLibErr << ","
                 << ind.ku << "," << kuIndErr << ","
                 << bias.ku << "," << kuBiasErr << ","
                 << indBias.ku << "," << kuBiasSteadyErr << ","
                 << lib.tu << "," << tuLibErr << ","
                 << bias.tu << "," << tuBiasErr << ","
                 << ind.dutyHigh << "\n";
        tableRelay << ps.id << " & "
                   << fmtSign(kuLibErr) << "\\% & " << fmtSign(kuIndErr) << "\\% & "
                   << fmtSign(kuBiasErr) << "\\% & " << fmtSign(kuBiasSteadyErr) << "\\% & "
                   << fmtSign(tuLibErr) << "\\% & " << fmtSign(tuBiasErr) << "\\% \\\\\n";
        summary << ps.id
                << " Ku_err lib=" << kuLibErr << "% indep(steady)=" << kuIndErr
                << "% bias=" << kuBiasErr << "% bias+steady=" << kuBiasSteadyErr << "%"
                << " Tu_err lib=" << tuLibErr << "% bias=" << tuBiasErr << "%"
                << " dutyHigh=" << ind.dutyHigh
                << " d=" << ind.relayD << "\n";
        }
        // Factorial main effects and interaction (signed errors, percentage points).
        const double mLib = mean(vLib), mSteady = mean(vSteady);
        const double mBias = mean(vBias), mBS = mean(vBS);
        const double effEstimator = mSteady - mLib;   // estimator switch, span relay
        const double effCentering = mBias - mLib;     // centering switch, lib estimator
        const double interaction = mBS - mSteady - mBias + mLib;
        summary << "factorial(Ku): mean lib=" << mLib << " steady=" << mSteady
                << " bias=" << mBias << " bias+steady=" << mBS
                << " | main effect estimator=" << effEstimator
                << "pp centering=" << effCentering << "pp interaction=" << interaction << "pp\n";
        summary << "factorial(Tu): mean lib=" << mean(vTuLib)
                << " bias=" << mean(vTuBias)
                << " | main effect centering=" << (mean(vTuBias) - mean(vTuLib)) << "pp\n";
    }
    tableRelay << "\\bottomrule\n\\end{tabular}\n";

    // ---- Section 2: Track A end-to-end comparison + multi-seed noise ----
    summary << "\n== Track A closed-loop comparison (Normal oscillation, library defaults) ==\n";
    const int kSeeds = 20;
    // Predeclared composite success criterion: settles within the 2% band,
    // overshoot <= 20%, mean absolute steady-state offset <= 2 units.
    auto successOf = [](const Metrics& m) {
        return m.settled && m.overshootPct <= 20.0 && m.essAbs <= 2.0;
    };
    // Failure classification by declared precedence, one primary label:
    // (1) oscillatory: outside the 5% band at the horizon; (2) overshoot:
    // exceeds the 20% limit; (3) offset: mean |e| > 2 units; (4) band:
    // none of the above, but the 2% band is not held.
    auto classify = [&](const Metrics& m) -> std::string {
        if (successOf(m)) return "OK";
        if (!m.settled5) return "oscillatory";
        if (m.overshootPct > 20.0) return "overshoot";
        if (m.essAbs > 2.0) return "offset";
        return "band";
    };
    auto tallyDominant = [](const std::vector<std::string>& v) {
        const char* names[5] = {"OK", "offset", "oscillatory", "band", "overshoot"};
        int cnt[5] = {0, 0, 0, 0, 0};
        for (const auto& s : v) {
            for (int i = 0; i < 5; ++i) if (s == names[i]) ++cnt[i];
        }
        int best = 0;
        for (int i = 1; i < 5; ++i) if (cnt[i] > cnt[best]) best = i;
        return std::string(names[best]);
    };
    std::vector<double> bandClean2, bandClean5, bandClean10;
    std::vector<double> bandN1_2, bandN1_5, bandN1_10;
    std::vector<double> bandN3_2, bandN3_5, bandN3_10;
    // Marginal per-requirement failure counts over the 25 clean runs.
    int margFailSettle = 0, margFailOvershoot = 0, margFailOffset = 0, margSuccess = 0;
    for (const auto& ps : kPlants) {
        double wc = 0.0, kuTrue = 0.0, tuTrue = 0.0;
        criticalPoint(ps, wc, kuTrue, tuTrue);
        for (auto m : methods) {
            const std::string tag = methodTag(m);
            const TuneResult tr = runTune(ps, m, atp::OscillationMode::Normal, 0.5, false);
            const bool wantFig = (ps.id == "P1" || ps.id == "P3");
            const std::string figPath = wantFig
                ? std::string(figDir) + "/step_" + ps.id + "_" + tag + ".csv" : "";
            const Metrics mc = runEval(ps, tr, false, 42U,
                                       wantFig ? figPath.c_str() : nullptr);

            // multi-seed noise: 1% and 3% of setpoint, kSeeds independent seeds
            std::vector<double> iae1, iae3;
            int succ1 = 0, s2_1 = 0, s5_1 = 0, s10_1 = 0;
            int s2_3 = 0, s5_3 = 0, s10_3 = 0;
            std::vector<std::string> classes1;
            for (int s = 1; s <= kSeeds; ++s) {
                Metrics m1 = runEval(ps, tr, true, static_cast<unsigned>(1000 + s), nullptr);
                iae1.push_back(m1.iae);
                succ1 += successOf(m1) ? 1 : 0;
                s2_1 += m1.settled ? 1 : 0;
                s5_1 += m1.settled5 ? 1 : 0;
                s10_1 += m1.settled10 ? 1 : 0;
                classes1.push_back(classify(m1));
                const Metrics m3 = runEval(ps, tr, true, static_cast<unsigned>(2000 + s),
                                           nullptr, true, 3.0);
                iae3.push_back(m3.iae);
                s2_3 += m3.settled ? 1 : 0;
                s5_3 += m3.settled5 ? 1 : 0;
                s10_3 += m3.settled10 ? 1 : 0;
            }
            const double iae1m = mean(iae1);
            const double iae1sd = stdev(iae1);
            const double iae3m = mean(iae3);
            const double iae3sd = stdev(iae3);
            const double succ1Pct = 100.0 * succ1 / kSeeds;
            noiseCsv << ps.id << "," << tag << ",1," << iae1m << "," << iae1sd << ","
                     << static_cast<double>(s2_1) / kSeeds << ","
                     << static_cast<double>(s5_1) / kSeeds << ","
                     << static_cast<double>(s10_1) / kSeeds << "\n";
            noiseCsv << ps.id << "," << tag << ",3," << iae3m << "," << iae3sd << ","
                     << static_cast<double>(s2_3) / kSeeds << ","
                     << static_cast<double>(s5_3) / kSeeds << ","
                     << static_cast<double>(s10_3) / kSeeds << "\n";

            bandClean2.push_back(mc.settled ? 1.0 : 0.0);
            bandClean5.push_back(mc.settled5 ? 1.0 : 0.0);
            bandClean10.push_back(mc.settled10 ? 1.0 : 0.0);
            bandN1_2.push_back(static_cast<double>(s2_1) / kSeeds);
            bandN1_5.push_back(static_cast<double>(s5_1) / kSeeds);
            bandN1_10.push_back(static_cast<double>(s10_1) / kSeeds);
            bandN3_2.push_back(static_cast<double>(s2_3) / kSeeds);
            bandN3_5.push_back(static_cast<double>(s5_3) / kSeeds);
            bandN3_10.push_back(static_cast<double>(s10_3) / kSeeds);

            // analytic anti-windup offset prediction for this tuning
            const double ePred = std::max(0.0, (kSp - tr.ki * kImax) / (1.0 + tr.kp));
            const std::string classClean = classify(mc);
            const std::string domNoise = tallyDominant(classes1);
            margFailSettle += mc.settled ? 0 : 1;
            margFailOvershoot += (mc.overshootPct > 20.0) ? 1 : 0;
            margFailOffset += (mc.essAbs > 2.0) ? 1 : 0;
            margSuccess += successOf(mc) ? 1 : 0;

            metricsCsv << ps.id << "," << tag << ",Normal,0.5," << tr.ok << ","
                       << tr.tuneTime << "," << tr.ku << "," << tr.tu << ","
                       << kuTrue << "," << tuTrue << ","
                       << tr.kp << "," << tr.ki << "," << tr.kd << ","
                       << mc.overshootPct << "," << mc.riseTime << "," << mc.settleTime << ","
                       << mc.settled << "," << mc.iae << "," << mc.ise << "," << mc.essAbs << ","
                       << ePred << "," << iae1m << "," << iae1sd << ","
                       << classClean << "," << succ1Pct << "," << mc.settled5 << ","
                       << mc.settled10 << "," << mc.essTailStd << "\n";
            iaeCsv << ps.id << "," << tag << "," << mc.iae << "," << iae1m << "\n";
            tableMain << ps.id << " & " << tag << " & "
                      << fmtG(tr.kp) << " & " << fmtG(tr.ki) << " & " << fmtG(tr.kd) << " & "
                      << fmtS(mc.overshootPct) << "\\% & " << fmtS(mc.riseTime) << " & "
                      << (mc.settled ? fmtS(mc.settleTime) : "$>$1800") << " & "
                      << fmtS(mc.iae) << " & " << fmtS(iae1m) << " & " << fmtS(mc.essAbs) << " \\\\\n";
            tableClass << ps.id << " & " << tag << " & " << classClean << " & "
                       << fmtS(succ1Pct) << "\\% & " << domNoise << " \\\\\n";
            summary << ps.id << " " << tag
                    << " Kp=" << tr.kp << " Ki=" << tr.ki << " Kd=" << tr.kd
                    << " OS=" << mc.overshootPct << "%"
                    << " tr=" << mc.riseTime << "s"
                    << " ts=" << (mc.settled ? std::to_string(mc.settleTime) : std::string(">1800"))
                    << " IAE=" << mc.iae
                    << " IAE1%=" << iae1m << "+-" << iae1sd
                    << " IAE3%=" << iae3m << "+-" << iae3sd
                    << " class=" << classClean
                    << " success@1%=" << succ1 << "/" << kSeeds
                    << " domclass@1%=" << domNoise
                    << " settled@1% 2/5/10%=" << s2_1 << "/" << s5_1 << "/" << s10_1
                    << " ess=" << mc.essAbs << " e_pred=" << ePred
                    << " tail_std=" << mc.essTailStd
                    << " tune=" << tr.tuneTime << "s"
                    << " ok=" << tr.ok << "\n";
        }
        if (ps.id != kPlants.back().id) {
            tableMain << "\\midrule\n";
            tableClass << "\\midrule\n";
        }
    }
    tableMain << "\\bottomrule\n\\end{tabular}\n";
    tableClass << "\\bottomrule\n\\end{tabular}\n";
    summary << "\n== Marginal requirement failures over 25 clean runs ==\n"
            << "composite success=" << margSuccess << "/25"
            << " | fail settling(2%)=" << margFailSettle
            << " fail overshoot(>20%)=" << margFailOvershoot
            << " fail offset(>2 units)=" << margFailOffset << "\n";

    {
        auto bandRow = [&](const char* band, const char* cond, std::vector<double>& v) {
            const double mn = mean(v) * 100.0;
            const double lo = *std::min_element(v.begin(), v.end()) * 100.0;
            const double hi = *std::max_element(v.begin(), v.end()) * 100.0;
            tableBand << band << " & " << cond << " & "
                      << fmtS(mn) << "\\% & " << fmtS(lo) << "\\% & " << fmtS(hi) << "\\% \\\\\n";
            summary << "band " << band << " " << cond
                    << ": mean " << mn << "% (min " << lo << ", max " << hi << ")\n";
        };
        summary << "\n== Settling-band sensitivity (settled fraction over 25 configurations) ==\n";
        bandRow("2\\%", "clean", bandClean2);
        bandRow("2\\%", "noise 1\\%", bandN1_2);
        bandRow("2\\%", "noise 3\\%", bandN3_2);
        bandRow("5\\%", "clean", bandClean5);
        bandRow("5\\%", "noise 1\\%", bandN1_5);
        bandRow("5\\%", "noise 3\\%", bandN3_5);
        bandRow("10\\%", "clean", bandClean10);
        bandRow("10\\%", "noise 1\\%", bandN1_10);
        bandRow("10\\%", "noise 3\\%", bandN3_10);
        tableBand << "\\bottomrule\n\\end{tabular}\n";
    }

    // ---- Section 2b: lambda sensitivity on P2 ----
    summary << "\n== Lambda sensitivity on P2 (T=10, L=5) ==\n";
    const PlantSpec* p2 = nullptr;
    for (const auto& ps : kPlants) {
        if (ps.id == "P2") p2 = &ps;
    }
    for (auto m : {atp::TuningMethod::IMC, atp::TuningMethod::LambdaTuning}) {
        for (double lam : {0.5, 10.0, 20.0}) {
            const TuneResult tr = runTune(*p2, m, atp::OscillationMode::Normal, lam, true);
            const Metrics mc = runEval(*p2, tr, false, 42U, nullptr);
            tableLambda << methodTag(m) << " & " << fmtS(lam) << " & "
                        << fmtG(tr.kp) << " & " << fmtG(tr.ki) << " & " << fmtG(tr.kd) << " & "
                        << fmtS(mc.overshootPct) << "\\% & " << fmtS(mc.riseTime) << " & "
                        << fmtS(mc.iae) << " \\\\\n";
            summary << p2->id << " " << methodTag(m) << " lambda=" << lam
                    << " Kp=" << tr.kp << " Ki=" << tr.ki << " Kd=" << tr.kd
                    << " OS=" << mc.overshootPct << "%"
                    << " tr=" << mc.riseTime << "s"
                    << " IAE=" << mc.iae << "\n";
        }
    }
    tableLambda << "\\bottomrule\n\\end{tabular}\n";

    // ---- Section 2c: Track B, tuning rules with ground-truth parameters ----
    // Anti-windup is disabled here so the rule comparison is not confounded by
    // the library's limiter defect (Section 2d isolates that mechanism).
    summary << "\n== Track B: rules with ground-truth models, AW off ==\n";
    {
        const std::vector<TrackBVariant> variants = {
            {"ZN*", 0, 0.5}, {"CC*", 1, 0.5}, {"TL*", 2, 0.5},
            {"IMC*$_{0.5}$", 3, 0.5}, {"IMC*$_{T}$", 4, 0.0}};
        for (const auto& ps : kPlants) {
        for (const auto& v : variants) {
            const TuneResult tr = trackBGains(ps, v);
            const Metrics mc = runEval(ps, tr, false, 42U, nullptr, false);
            const Metrics mcAW = runEval(ps, tr, false, 42U, nullptr, true);
            trackBCsv << ps.id << "," << v.tag << "," << tr.kp << "," << tr.ki << ","
                      << tr.kd << "," << mc.overshootPct << "," << mc.riseTime << ","
                      << mc.settleTime << "," << mc.settled << "," << mc.iae << ","
                      << mc.essAbs << "," << mcAW.essAbs << "\n";
            tableTrackB << ps.id << " & " << v.tag << " & "
                        << fmtG(tr.kp) << " & " << fmtG(tr.ki) << " & " << fmtG(tr.kd) << " & "
                        << fmtS(mc.overshootPct) << "\\% & " << fmtS(mc.riseTime) << " & "
                        << (mc.settled ? fmtS(mc.settleTime) : "$>$1800") << " & "
                        << fmtS(mc.iae) << " & " << fmtS(mc.essAbs) << " & "
                        << fmtS(mcAW.essAbs) << " \\\\\n";
                summary << ps.id << " " << v.tag
                        << " Kp=" << tr.kp << " Ki=" << tr.ki << " Kd=" << tr.kd
                        << " OS=" << mc.overshootPct << "%"
                        << " tr=" << mc.riseTime << "s"
                        << " ts=" << (mc.settled ? std::to_string(mc.settleTime) : std::string(">1800"))
                        << " IAE=" << mc.iae << " ess=" << mc.essAbs << "\n";
            }
            if (ps.id != kPlants.back().id) {
                tableTrackB << "\\midrule\n";
            }
        }
    }
    tableTrackB << "\\bottomrule\n\\end{tabular}\n";

    // ---- Section 2d: anti-windup mechanism audit and ablation ----
    summary << "\n== Anti-windup audit: predicted vs observed offset, AW on/off ==\n";
    {
        struct AwCase { const PlantSpec* ps; std::string tag; double lambda; bool useLam; };
        const std::vector<AwCase> cases = {
            {p2, "IMC", 0.5, false}, {p2, "TL", 0.5, false},
            {&kPlants[2], "TL", 0.5, false}, {&kPlants[3], "IMC", 0.5, false},
            {p2, "IMC", 10.0, true}};
        for (const auto& c : cases) {
            const atp::TuningMethod m = (c.tag == "IMC") ? atp::TuningMethod::IMC
                                                          : atp::TuningMethod::TyreusLuyben;
            const TuneResult tr = runTune(*c.ps, m, atp::OscillationMode::Normal,
                                          c.lambda, c.useLam);
            const Metrics on = runEval(*c.ps, tr, false, 42U, nullptr, true);
            const Metrics off = runEval(*c.ps, tr, false, 42U, nullptr, false);
            const Metrics rep = runEval(*c.ps, tr, false, 42U, nullptr, true, 1.0, true);
            const double iEq = kSp / tr.ki;
            const double ePred = std::max(0.0, (kSp - tr.ki * kImax) / (1.0 + tr.kp));
            const std::string lamTag = c.useLam ? (", $\\lambda{=}$" + fmtS(c.lambda)) : "";
            awCsv << c.ps->id << "," << c.tag << "," << tr.ki << "," << iEq << ","
                  << ePred << "," << on.essAbs << "," << off.essAbs << "," << rep.essAbs << "\n";
            tableAW << c.ps->id << " & " << c.tag << lamTag << " & "
                    << fmtG(tr.ki) << " & " << fmtS(iEq) << " & "
                    << fmtS(ePred) << " & " << fmtS(on.essAbs) << " & "
                    << fmtS(off.essAbs) << " & " << fmtS(rep.essAbs) << " \\\\\n";
            summary << c.ps->id << " " << c.tag << (c.useLam ? " lam=10" : "")
                    << " Ki=" << tr.ki << " Ieq=" << iEq
                    << " e_pred=" << ePred
                    << " e_obs_on=" << on.essAbs
                    << " e_obs_off=" << off.essAbs << "\n";
        }
    }
    tableAW << "\\bottomrule\n\\end{tabular}\n";

    // ---- Section 2e: plant-gain sensitivity sweep on P2 dynamics ----
    summary << "\n== Plant-gain sweep on P2 dynamics (K=0.5, K=2), Track A ==\n";
    {
        PlantSpec p05 = *p2; p05.id = "P2"; p05.K = 0.5;
        PlantSpec p20 = *p2; p20.id = "P2"; p20.K = 2.0;
        for (auto m : methods) {
            const TuneResult t05 = runTune(p05, m, atp::OscillationMode::Normal, 0.5, false);
            const Metrics m05 = runEval(p05, t05, false, 42U, nullptr);
            const TuneResult t20 = runTune(p20, m, atp::OscillationMode::Normal, 0.5, false);
            const Metrics m20 = runEval(p20, t20, false, 42U, nullptr);
            gainCsv << methodTag(m) << ",0.5," << m05.overshootPct << "," << m05.essAbs
                    << "," << m05.iae << "\n";
            gainCsv << methodTag(m) << ",2.0," << m20.overshootPct << "," << m20.essAbs
                    << "," << m20.iae << "\n";
            tableGain << methodTag(m) << " & "
                      << fmtS(m05.overshootPct) << "\\% & " << fmtS(m05.essAbs) << " & "
                      << fmtS(m20.overshootPct) << "\\% & " << fmtS(m20.essAbs) << " \\\\\n";
            summary << methodTag(m)
                    << " K=0.5 OS=" << m05.overshootPct << "% ess=" << m05.essAbs
                    << " IAE=" << m05.iae
                    << " | K=2.0 OS=" << m20.overshootPct << "% ess=" << m20.essAbs
                    << " IAE=" << m20.iae << "\n";
        }
    }
    tableGain << "\\bottomrule\n\\end{tabular}\n";

    // ---- Section 3: resource footprint ----
    const double nsPerCall = benchUpdateNs();
    summary << "\n== Resource footprint (host x86-64, -O2) ==\n";
    summary << "sizeof(AutoTunePID) = " << sizeof(atp::AutoTunePID) << " bytes\n";
    summary << "update(input, dt) = " << nsPerCall << " ns/call (Normal mode)\n";

    std::ofstream resTxt(std::string(outDir) + "/footprint.txt");
    resTxt << sizeof(atp::AutoTunePID) << " " << nsPerCall << "\n";

    summary << "\nDone.\n";
    return 0;
}
