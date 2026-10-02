#ifndef AIC_ENGINE_SHELL_UTILS_HPP_
#define AIC_ENGINE_SHELL_UTILS_HPP_

#include <optional>
#include <string>

namespace aic {

// Encode exactly one argument for the POSIX shell used by popen().
// NUL cannot be represented in a process argument.
inline std::optional<std::string> QuoteShellArgument(const std::string& value) {
  if (value.find('\0') != std::string::npos) {
    return std::nullopt;
  }
  std::string quoted = "'";
  for (char character : value) {
    if (character == '\'') {
      quoted += "'\\''";
    } else {
      quoted += character;
    }
  }
  quoted += "'";
  return quoted;
}

}  // namespace aic

#endif  // AIC_ENGINE_SHELL_UTILS_HPP_
