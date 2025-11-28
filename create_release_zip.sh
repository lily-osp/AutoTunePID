#!/bin/bash

# AutoTunePID Library - Create Arduino Release ZIP
# This script creates a ZIP file with the proper Arduino library structure

set -e

echo "🔧 AutoTunePID Library - Release ZIP Creator"
echo "=============================================="

# Get version from library.properties
if [ ! -f "library.properties" ]; then
    echo "❌ Error: library.properties not found!"
    exit 1
fi

VERSION=$(grep "^version=" library.properties | cut -d'=' -f2)
if [ -z "$VERSION" ]; then
    echo "❌ Error: Could not find version in library.properties"
    exit 1
fi

echo "📦 Creating release for version: $VERSION"

# Create temporary directory for Arduino library structure
TEMP_DIR="arduino-lib-$VERSION"
ZIP_NAME="AutoTunePID-$VERSION.zip"

echo "📁 Creating Arduino library structure..."

# Clean up any existing temp directory
rm -rf "$TEMP_DIR"
mkdir "$TEMP_DIR"

# Copy required files
echo "📋 Copying library files..."

# Core library files
cp -r src "$TEMP_DIR/"
cp library.properties "$TEMP_DIR/"
cp keywords.txt "$TEMP_DIR/"
cp LICENSE "$TEMP_DIR/"
cp README.md "$TEMP_DIR/"

# Examples
if [ -d "examples" ]; then
    cp -r examples "$TEMP_DIR/"
    echo "✅ Examples copied"
else
    echo "⚠️  Warning: No examples directory found"
fi

# Validate structure
echo "🔍 Validating library structure..."

REQUIRED_FILES=("src/AutoTunePID.h" "src/AutoTunePID.cpp" "library.properties" "keywords.txt" "LICENSE")
for file in "${REQUIRED_FILES[@]}"; do
    if [ ! -f "$TEMP_DIR/$file" ]; then
        echo "❌ Missing required file: $file"
        rm -rf "$TEMP_DIR"
        exit 1
    fi
done

# Check for README (either name)
if [ ! -f "$TEMP_DIR/README.md" ]; then
    echo "❌ Missing README.md file"
    rm -rf "$TEMP_DIR"
    exit 1
fi

echo "✅ All required files present"

# Create ZIP file
echo "📦 Creating ZIP file: $ZIP_NAME"
cd "$TEMP_DIR"
zip -r "../$ZIP_NAME" .

# Verify ZIP contents
echo "📋 ZIP contents:"
unzip -l "../$ZIP_NAME"

# Clean up
cd ..
rm -rf "$TEMP_DIR"

echo ""
echo "🎉 Success! Created $ZIP_NAME"
echo ""
echo "📝 Next steps:"
echo "1. Test the ZIP: arduino-cli lib install --zip-path $ZIP_NAME"
echo "2. For release: Create a Git tag and push it"
echo "   git tag v$VERSION"
echo "   git push origin v$VERSION"
echo "3. GitHub Actions will automatically create the release"

echo ""
echo "📦 ZIP file ready for Arduino IDE installation!"
