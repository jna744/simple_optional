#include <simple/optional.hpp>

#include <functional>
#include <iostream>

struct S {
  S() {}
  S(S const&){};
  ~S() {}
  int    x;
  int    y;
  double f;
};

#include <string>

int main()
{

  using namespace simple;

  optional<S> other_opt = S{};

  auto transformed = other_opt.and_then([](auto& s) { return optional<int>{80}; })
                         .and_then([](auto i) { return optional<std::string>{"Hello"}; })
                         .transform(&std::string::size);

  std::cout << "transformed has value: " << transformed.has_value() << std::endl;
  std::cout << "transformed value: " << transformed.value_or(0) << std::endl;
}
