#!/usr/bin/env bash

cd "$(dirname "$0")"

K="$(uname -s | tr '[:upper:]' '[:lower:]')"
"../../bin/$K-$(uname -m)/ek9000Test" st3.cmd
