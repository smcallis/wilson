#!/usr/bin/env sh

# Copyright 2024 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

if [ $# -lt 3 ]; then
   echo "Error, invalid arguments" >& 2
   exit 1
fi

input=$1
output=$2
symbol=$3
namespace=$4

mkdir -p "$(dirname "${output}")"

# Make sure the file exists and write the namespace if requested
cat <<EOF > "${output}"
#include <cstdlib>

EOF

if [ -n "${namespace}" ]; then
    echo "namespace ${namespace} {" >> "${output}"
fi

# Write the embedded string literal.
{
    echo "extern const char _resource_${symbol}[] = \\";

    hexdump -v -e '16/1 "_x%02X" "\n"' "${input}" \
        | sed 's/_/\\/g; s/\\x  //g; s/.*/    "&"/';

    echo ";";
    echo "extern const size_t _resource_${symbol}_size = sizeof(_resource_${symbol})-1;"
} >> "${output}"

# Close the namespace (if any).
if [ -n "${namespace}" ]; then
    echo "}  // namespace ${namespace}" >> "${output}"
fi
