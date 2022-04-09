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

simple::optional<S> make_optional(unsigned char i)
{
  return S{};
}

#include <string>

int main()
{

  using namespace simple;

  S s;

  auto& f = optional_ns::invoke(&S::f, s);

  auto        opt = make_optional(35);
  optional<S> other_opt;

  auto transformed = other_opt.and_then([](auto& s) { return optional<int>{}; })
                         .and_then([](auto i) { return optional<std::string>{"Hello"}; })
                         .transform([](auto str) { return 10; });

  other_opt = opt;
  std::cout << "opt has value: " << opt.has_value() << std::endl;
  std::cout << "other_opt has value: " << other_opt.has_value() << std::endl;
}
