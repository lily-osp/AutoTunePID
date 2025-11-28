# GitHub Actions Workflows

This directory contains GitHub Actions workflows for automated validation, testing, and release creation of the AutoTunePID Arduino library.

## Workflows Overview

### 1. `validate.yml` - Library Validation
**Triggers:** Push/PR to main branch
**Purpose:** Validates Arduino library structure and compiles all examples

**Checks performed:**
- ✅ Required files present (`src/`, `library.properties`, `keywords.txt`, `LICENSE`, `README.md`)
- ✅ Examples directory exists
- ✅ `library.properties` format validation
- ✅ `keywords.txt` format validation
- ✅ Semantic versioning format
- ✅ Emoji-free documentation
- ✅ All examples compile successfully

### 2. `release.yml` - Manual Release Creation
**Triggers:** When a GitHub release is published
**Purpose:** Creates and attaches Arduino library ZIP to releases

**Actions:**
- Extracts version from `library.properties`
- Creates Arduino-compatible ZIP structure
- Uploads ZIP as release asset

### 3. `auto-release.yml` - Automatic Release Creation
**Triggers:** Version tags pushed (e.g., `v1.2.3`)
**Purpose:** Fully automated release creation from version tags

**Actions:**
- Validates tag matches `library.properties` version
- Creates GitHub release with proper description
- Generates Arduino library ZIP
- Uploads ZIP to release

### 4. `manual-release.yml` - Manual Release Workflow
**Triggers:** Manual workflow dispatch
**Purpose:** Create releases on-demand with version input

**Features:**
- Input version number manually
- Option to mark as pre-release
- Automatically updates `library.properties`
- Creates git tag
- Generates release and ZIP

## Usage Instructions

### Automatic Releases (Recommended)
1. Update version in `library.properties`
2. Commit changes: `git commit -m "Bump version to x.x.x"`
3. Create and push tag: `git tag vx.x.x && git push origin vx.x.x`
4. GitHub Actions creates release automatically

### Manual Releases
1. Go to GitHub Actions tab
2. Select "Manual Release Creation" workflow
3. Click "Run workflow"
4. Enter version number and pre-release option
5. Workflow handles everything automatically

### Local Testing
Use the `create_release_zip.sh` script for local ZIP creation:

```bash
./create_release_zip.sh
```

This creates a properly structured Arduino library ZIP for testing.

## Arduino Library ZIP Structure

The workflows create ZIP files with this structure:

```
AutoTunePID-x.x.x/
├── src/
│   ├── AutoTunePID.h
│   └── AutoTunePID.cpp
├── examples/
│   ├── ZieglerNichols/
│   ├── CohenCoon/
│   ├── IMC/
│   ├── TyreusLuyben/
│   ├── LambdaTuning/
│   └── Manual/
├── library.properties
├── keywords.txt
├── LICENSE
└── README.md
```

## Requirements

- GitHub repository with Actions enabled
- `library.properties` with valid semantic version
- Proper Arduino library structure
- All examples must compile successfully

## Troubleshooting

### Validation Failures
- Check that all required files exist
- Ensure `keywords.txt` uses tabs (not spaces)
- Verify version follows semantic versioning (x.x.x)

### Release Issues
- Confirm tag matches `library.properties` version exactly
- Check that GitHub Actions has proper permissions
- Verify all examples compile before releasing

### ZIP Structure Issues
- Run `./create_release_zip.sh` locally to test
- Check file permissions
- Ensure no special characters in filenames
