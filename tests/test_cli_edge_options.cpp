#include <catch2/catch_test_macros.hpp>
#include <string>
#include <vector>

#include "options.h"

TEST_CASE("CLI accepts bounded edge evidence options and rejects removed policy selection",
          "[cli][vecr]") {
    using swarmkit::apps::cli::internal::ParseInvocation;
    std::vector<std::string> args{"swarmkit-cli",
                                  "edge",
                                  "execute",
                                  "--capture-age-ms",
                                  "3000",
                                  "--evidence-wait-ms",
                                  "3000",
                                  "--hard-speed-bound",
                                  "5",
                                  "--recovery-attempts",
                                  "2"};
    std::vector<char*> argv;
    for (auto& value : args)
        argv.push_back(value.data());
    const auto invocation = ParseInvocation(static_cast<int>(argv.size()), argv.data());
    REQUIRE(invocation);
    CHECK(invocation->command == "edge");
    CHECK_FALSE(invocation->has_explicit_address);
    args = {"swarmkit-cli", "edge", "execute", "--repair-policy", "evidence-aware"};
    argv.clear();
    for (auto& value : args)
        argv.push_back(value.data());
    CHECK_FALSE(ParseInvocation(static_cast<int>(argv.size()), argv.data()));
}
