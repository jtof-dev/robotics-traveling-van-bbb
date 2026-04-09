#!/bin/bash

# ==========================================
# DEPENDENCY CONFIGURATION
# script written by google gemini
# Format: "local/folder/path | https://github.com/... | optional_branch"
# ==========================================
LIBS=(
    "lib/pico-sdk | https://github.com/raspberrypi/pico-sdk.git"
    "lib/VL53L0X  | https://github.com/yspreen/VL53L0X-driver-pico-sdk-cpp.git"
    "lib/pid | https://github.com/jtof-dev/pico-pid-library.git"
)

pwd=$(pwd)
repoRoot=$(git rev-parse --show-toplevel)

cd "$repoRoot"

for entry in "${LIBS[@]}"; do
    # Split the string by the pipe symbol "|"
    IFS='|' read -r raw_folder raw_url raw_branch <<< "$entry"
    
    # Trim leading/trailing whitespace
    folder=$(echo "$raw_folder" | xargs)
    url=$(echo "$raw_url" | xargs)
    branch=$(echo "$raw_branch" | xargs)

    echo "----------------------------------------"
    echo "Scrubbing: $folder"
    echo "----------------------------------------"

    # 1. Un-cache from Git's memory
    git rm -r --cached "$folder" 2>/dev/null

    # 2. Delete the physical folder
    rm -rf "$folder"

    # 3. Delete Git's hidden internal tracking data
    rm -rf ".git/modules/$folder"

    # 4. Strip it completely out of Git's config files
    git config --remove-section "submodule.$folder" 2>/dev/null
    git config -f .gitmodules --remove-section "submodule.$folder" 2>/dev/null

    echo "Re-adding submodule to local folder: [$folder]"
    echo "Fetching from: $url"

    # 5. Force add the submodule back cleanly
    if [ -n "$branch" ]; then
        echo "Target branch: $branch"
        git submodule add -f -b "$branch" "$url" "$folder"
    else
        git submodule add -f "$url" "$folder"
    fi
done

echo "----------------------------------------"
echo "Fetching all nested dependencies..."
echo "----------------------------------------"
git submodule update --init --recursive

cd "$pwd"
