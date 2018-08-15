#!/bin/bash
git add .
guidance_version='cat ./carmajava/guidance/src/main/java/gov/dot/fhwa/saxton/carma/guidance/CarmaVersion.java'
major=$($guidance_version | grep 'int major' | grep -o '[0-9]*')
intermediate=$($guidance_version | grep 'int intermediate' | grep -o '[0-9]*')
minor=$($guidance_version | grep 'int minor' | grep -o '[0-9]*')
current_commit_short_hash=$(git rev-parse --short HEAD)
cd carmajava
hide=$(./gradlew :guidance:writeVersionFile)
cd ./guidance/src/main/resources/
versionCode=$(sed -n 1p version)
build_suffix=$(sed -n 2p version)
echo "The predicted build name is "$major'.'$intermediate'.'$minor'.'$versionCode'-'$build_suffix
echo "The shorthand sha hash of the latest commit of the branch you are currently on is "$current_commit_short_hash
