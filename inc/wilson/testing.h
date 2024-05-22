#pragma once

// Copyright 2024 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// The DOCTEST macro names conflict with Abseil logging names.  We'll define
// them ourselves here.
#define DOCTEST_CONFIG_NO_SHORT_MACRO_NAMES
#define DOCTEST_CONFIG_SUPER_FAST_ASSERTS

#include <third_party/doctest.h>

#ifndef WILSON_TEST_MACROS
#define WILSON_TEST_MACROS

#define TEST_CASE              DOCTEST_TEST_CASE
#define TEST_CASE_CLASS        DOCTEST_TEST_CASE_CLASS
#define TEST_CASE_FIXTURE      DOCTEST_TEST_CASE_FIXTURE
#define TEST_CASE_TEMPLATE     DOCTEST_TEST_CASE_TEMPLATE
#define SUBCASE                DOCTEST_SUBCASE
#define TEST_SUITE             DOCTEST_TEST_SUITE
#define TEST_SUITE_BEGIN       DOCTEST_TEST_SUITE_BEGIN
#define TEST_SUITE_END         DOCTEST_TEST_SUITE_END

#define SCENARIO               DOCTEST_SCENARIO
#define GIVEN                  DOCTEST_GIVEN
#define WHEN                   DOCTEST_WHEN
#define AND_WHEN               DOCTEST_AND_WHEN
#define THEN                   DOCTEST_THEN
#define AND_THEN               DOCTEST_AND_THEN

#define TCHECK                 DOCTEST_CHECK
#define TCHECK_FALSE           DOCTEST_CHECK_FALSE
#define TCHECK_THROWS          DOCTEST_CHECK_THROWS
#define TCHECK_THROWS_AS       DOCTEST_CHECK_THROWS_AS
#define TCHECK_THROWS_WITH     DOCTEST_CHECK_THROWS_WITH
#define TCHECK_THROWS_WITH_AS  DOCTEST_CHECK_THROWS_WITH_AS
#define TCHECK_NOTHROW         DOCTEST_CHECK_NOTHROW
#define TCHECK_EQ              DOCTEST_CHECK_EQ
#define TCHECK_NE              DOCTEST_CHECK_NE
#define TCHECK_GT              DOCTEST_CHECK_GT
#define TCHECK_LT              DOCTEST_CHECK_LT
#define TCHECK_GE              DOCTEST_CHECK_GE
#define TCHECK_LE              DOCTEST_CHECK_LE
#define TCHECK_UNARY           DOCTEST_CHECK_UNARY
#define TCHECK_UNARY_FALSE     DOCTEST_CHECK_UNARY_FALSE

#define REQUIRE                DOCTEST_REQUIRE
#define REQUIRE_FALSE          DOCTEST_REQUIRE_FALSE
#define REQUIRE_THROWS         DOCTEST_REQUIRE_THROWS
#define REQUIRE_THROWS_AS      DOCTEST_REQUIRE_THROWS_AS
#define REQUIRE_THROWS_WITH_AS DOCTEST_REQUIRE_THROWS_WITH_AS
#define REQUIRE_NOTHROW        DOCTEST_REQUIRE_NOTHROW
#define REQUIRE_EQ             DOCTEST_REQUIRE_EQ
#define REQUIRE_NE             DOCTEST_REQUIRE_NE
#define REQUIRE_GT             DOCTEST_REQUIRE_GT
#define REQUIRE_LT             DOCTEST_REQUIRE_LT
#define REQUIRE_GE             DOCTEST_REQUIRE_GE
#define REQUIRE_LE             DOCTEST_REQUIRE_LE
#define REQUIRE_UNARY          DOCTEST_REQUIRE_UNARY
#define REQUIRE_UNARY_FALSE    DOCTEST_REQUIRE_UNARY_FALSE

#define WARN                   DOCTEST_WARN
#define WARN_FALSE             DOCTEST_WARN_FALSE
#define WARN_THROWS            DOCTEST_WARN_THROWS
#define WARN_THROWS_AS         DOCTEST_WARN_THROWS_AS
#define WARN_THROWS_WITH       DOCTEST_WARN_THROWS_WITH
#define WARN_THROWS_WITH_AS    DOCTEST_WARN_THROWS_WITH_AS
#define WARN_NOTHROW           DOCTEST_WARN_NOTHROW
#define WARN_EQ                DOCTEST_WARN_EQ
#define WARN_NE                DOCTEST_WARN_NE
#define WARN_GT                DOCTEST_WARN_GT
#define WARN_LT                DOCTEST_WARN_LT
#define WARN_GE                DOCTEST_WARN_GE
#define WARN_LE                DOCTEST_WARN_LE
#define WARN_UNARY             DOCTEST_WARN_UNARY
#define WARN_UNARY_FALSE       DOCTEST_WARN_UNARY_FALSE

#endif // WILSON_TEST_MACROS



