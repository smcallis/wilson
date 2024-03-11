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

output=$1
symbol=$2
getter=$3
namespace=$4

mkdir -p "$(dirname "${output}")"

# Write includes and extern the variables we need.
cat <<EOF > "${output}"
#include "absl/types/span.h"

EOF

if [ -n "${namespace}" ]; then
    echo "namespace ${namespace} {" >> "${output}"
fi

cat <<EOF >> "${output}"
extern const char _resource_${symbol}[];
extern const size_t _resource_${symbol}_size;

inline absl::Span<const char> ${getter}() {
  return {_resource_${symbol}, _resource_${symbol}_size};
}
EOF

# Close the namespace (if any).
if [ -n "${namespace}" ]; then
    cat <<EOF >> "${output}"
}  // namespace ${namespace}
EOF
fi
