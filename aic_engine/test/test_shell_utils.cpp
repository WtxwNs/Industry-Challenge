#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

#include "shell_utils.hpp"

namespace {
void Check(bool condition) {
  if (!condition) {
    throw std::runtime_error("Shell argument regression check failed");
  }
}
}  // namespace

int main() {
  const std::vector<std::string> values = {
      "",
      "simple",
      "path with spaces/world.xacro",
      "a'b",
      "line\nbreak",
      "cable_type:=$UNSET_VARIABLE; * ? | & < > ( ) \" \\ ",
  };
  for (const auto& value : values) {
    const auto quoted = aic::QuoteShellArgument(value);
    Check(quoted.has_value());
    // A harmless shell round-trip verifies that the argument stays literal.
    const std::string command = "printf '%s' " + *quoted;
    FILE* pipe = popen(command.c_str(), "r");
    Check(pipe != nullptr);
    std::string output;
    char buffer[128];
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
      output += buffer;
    }
    Check(pclose(pipe) == 0);
    Check(output == value);
  }
  Check(!aic::QuoteShellArgument(std::string("a\0b", 3)).has_value());
  return 0;
}
