#pragma once

namespace swarmkit::core {

/**
 * @brief Visitor helper for std::visit with multiple lambda overloads.
 *
 * Combines several callable objects into one overload set so that
 * std::visit can dispatch to the correct handler per variant alternative
 * without chains of if-constexpr.
 *
 * @par Example
 * @code
 * std::visit(core::Overloaded{
 *     [&](const CmdArm&)            { ... },
 *     [&](const CmdDisarm&)         { ... },
 *     [&](const CmdTakeoff& cmd)    { ... },
 *     [&](const CmdLand&)           { ... },
 *     [&](const CmdSetRole& cmd)    { ... },
 * }, envelope.command);
 * @endcode
 *
 * Adding a new command type to the Command variant without providing an
 * overload here produces a compile error, enforcing exhaustive handling.
 */
template <typename... Ts>
struct Overloaded : Ts... {
    using Ts::operator()...;
};

}  // namespace swarmkit::core
