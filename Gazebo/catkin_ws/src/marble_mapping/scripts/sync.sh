#!/bin/bash
git remote add upstream https://github.com/dan-riley/marble_mapping
git fetch upstream
git checkout master
git reset --hard upstream/master
# git rebase upstream/master
git push origin master --force
