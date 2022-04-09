#ifndef SIMPLE_OPTIONAL_HPP
#define SIMPLE_OPTIONAL_HPP

#include <exception>
#include <type_traits>
#include <utility>

#if __cplusplus >= 201703L

#define SOPT_INLINE_17 inline
#define SOPT_INLINE_14

#elif __cplusplus >= 201402L

#define SOPT_INLINE_17
#define SOPT_INLINE_14 inline

#else

#error "simple::optional requires at least c++14"

#endif

#define SOPT_INLINE(version) SOPT_INLINE_##version

namespace std
{
// fwd
template <typename>
struct hash;
}  // namespace std

namespace simple
{

// fwd
template <typename T>
class optional;

class bad_optional_access : public std::exception
{
public:

  bad_optional_access() = default;
  virtual ~bad_optional_access() = default;
  char const* what() const noexcept override { return "bad_optional_access"; }
};

struct nullopt_t {
};

SOPT_INLINE(17) constexpr auto nullopt = nullopt_t{};

struct in_place_t {
};

SOPT_INLINE(17) constexpr auto in_place = in_place_t{};

namespace optional_ns
{

template <typename T>
auto declval() noexcept -> std::add_rvalue_reference_t<T>;

namespace swappable_impl
{

using std::swap;

template <typename T, typename U, typename = void>
struct is_swappable_with : std::false_type {
};

template <typename T, typename U>
struct is_swappable_with<T, U, decltype(swap(declval<T>(), declval<U>()))> : std::true_type {
};

template <typename T, typename U, typename = void>
struct is_nothrow_swappable_with : std::false_type {
};

template <typename T, typename U>
struct is_nothrow_swappable_with<T, U, std::enable_if_t<noexcept(swap(declval<T>(), declval<U>()))>> : std::true_type {
};

}  // namespace swappable_impl

template <typename T, typename U>
struct is_swappable_with : swappable_impl::is_swappable_with<T, U>::type {
};

template <typename T>
struct is_swappable : is_swappable_with<T&, T&>::type {
};

template <typename T, typename U>
struct is_nothrow_swappable_with : swappable_impl::is_nothrow_swappable_with<T, U>::type {
};

template <typename T>
struct is_nothrow_swappable : is_nothrow_swappable_with<T&, T&>::type {
};

template <typename...>
struct m_all_of : std::true_type {
};

template <typename T, typename... Ts>
struct m_all_of<T, Ts...> : std::conditional_t<bool(T::value), typename m_all_of<Ts...>::type, std::false_type> {
};

template <typename...>
struct m_any_of : std::false_type {
};

template <typename T, typename... Ts>
struct m_any_of<T, Ts...> : std::conditional_t<bool(T::value), std::true_type, typename m_any_of<Ts...>::type> {
};

template <typename T>
using m_not = std::integral_constant<bool, !bool(T::value)>;

template <typename T>
struct is_optional : std::false_type {
};

template <typename T>
struct is_optional<optional<T>> : std::true_type {
};

// CPP 17 invoke implementation

template <typename>
SOPT_INLINE(17)
constexpr bool is_reference_wrapper_v = false;

template <typename T>
SOPT_INLINE(17)
constexpr bool is_reference_wrapper_v<std::reference_wrapper<T>> = false;

struct invoke_mem_fn_tag_1 {
};

struct invoke_mem_fn_tag_2 {
};

struct invoke_mem_fn_tag_3 {
};

template <typename C, typename O>
using invoke_mem_fn_tag = std::conditional_t<
    std::is_base_of<C, O>::value,
    invoke_mem_fn_tag_1,
    std::conditional_t<is_reference_wrapper_v<O>, invoke_mem_fn_tag_2, invoke_mem_fn_tag_3>>;

template <typename C, typename P, typename O, typename... Args>
inline constexpr decltype(auto) invoke_mem_ptr(invoke_mem_fn_tag_1, P C::*ptr, O&& o, Args&&... args) noexcept(
    noexcept((std::forward<O>(o).*ptr)(std::forward<Args>(args)...)))
{
  return (std::forward<O>(o).*ptr)(std::forward<Args>(args)...);
}

template <typename C, typename P, typename O, typename... Args>
inline constexpr decltype(auto) invoke_mem_ptr(invoke_mem_fn_tag_2, P C::*ptr, O&& o, Args&&... args) noexcept(
    noexcept((o.get().*ptr)(std::forward<Args>(args)...)))
{
  return (o.get().*ptr)(std::forward<Args>(args)...);
}

template <typename C, typename P, typename O, typename... Args>
inline constexpr decltype(auto) invoke_mem_ptr(invoke_mem_fn_tag_3, P C::*ptr, O&& o, Args&&... args) noexcept(
    noexcept(((*std::forward<O>(o)).*ptr)(std::forward<Args>(args)...)))
{
  return ((*std::forward<O>(o)).*ptr)(std::forward<Args>(args)...);
}

struct invoke_mem_ptr_tag_1 {
};

struct invoke_mem_ptr_tag_2 {
};

struct invoke_mem_ptr_tag_3 {
};

template <typename C, typename O>
using invoke_mem_ptr_tag = std::conditional_t<
    std::is_base_of<C, O>::value,
    invoke_mem_ptr_tag_1,
    std::conditional_t<is_reference_wrapper_v<O>, invoke_mem_ptr_tag_2, invoke_mem_ptr_tag_3>>;

template <typename C, typename P, typename O>
inline constexpr decltype(auto)
invoke_mem_ptr(invoke_mem_ptr_tag_1, P C::*ptr, O&& o) noexcept(noexcept(std::forward<O>(o).*ptr))
{
  return std::forward<O>(o).*ptr;
}

template <typename C, typename P, typename O>
inline constexpr decltype(auto) invoke_mem_ptr(invoke_mem_ptr_tag_2, P C::*ptr, O&& o) noexcept(noexcept(o.get().*ptr))
{
  return o.get().*ptr;
}

template <typename C, typename P, typename O>
inline constexpr decltype(auto)
invoke_mem_ptr(invoke_mem_ptr_tag_3, P C::*ptr, O&& o) noexcept(noexcept((*std::forward<O>(o)).*ptr))
{
  return (*std::forward<O>(o)).*ptr;
}

template <typename C, typename P, typename O>
using invoke_member_tag = std::conditional_t<
    std::is_function<P>::value,
    invoke_mem_fn_tag<C, std::decay_t<O>>,
    invoke_mem_ptr_tag<C, std::decay_t<O>>>;

template <typename C, typename P, typename O, typename... Args>
inline constexpr decltype(auto) invoke_member_impl(P C::*ptr, O&& o, Args&&... args) noexcept(
    noexcept(invoke_mem_ptr(invoke_member_tag<C, P, O>{}, ptr, std::forward<O>(o), std::forward<Args>(args)...)))
{
  return invoke_mem_ptr(invoke_member_tag<C, P, O>{}, ptr, std::forward<O>(o), std::forward<Args>(args)...);
}

template <typename F, typename... Args>
inline constexpr decltype(auto) invoke_impl(std::true_type, F&& fn, Args&&... args) noexcept(
    noexcept(invoke_member_impl(std::forward<F>(fn), std::forward<Args>(args)...)))
{
  return invoke_member_impl(std::forward<F>(fn), std::forward<Args>(args)...);
}

template <typename F, typename... Args>
inline constexpr decltype(auto) invoke_impl(std::false_type, F&& fn, Args&&... args) noexcept(
    noexcept(std::forward<F>(fn)(std::forward<Args>(args)...)))
{
  return std::forward<F>(fn)(std::forward<Args>(args)...);
}

template <typename F, typename... Args>
inline constexpr decltype(auto) invoke(F&& fn, Args&&... args) noexcept(
    noexcept(invoke_impl(std::is_member_pointer<std::decay_t<F>>{}, std::forward<F>(fn), std::forward<Args>(args)...)))
{
  return invoke_impl(std::is_member_pointer<std::decay_t<F>>{}, std::forward<F>(fn), std::forward<Args>(args)...);
}

template <typename F, typename... Args>
struct invoke_result {
  using type = decltype(optional_ns::invoke(declval<F>(), declval<Args>()...));
};

template <typename F, typename... Args>
using invoke_result_t = typename invoke_result<F, Args...>::type;

template <typename, typename F, typename... Args>
struct is_invocable_impl : std::false_type {
};

template <typename F, typename... Args>
struct is_invocable_impl<void(invoke_result_t<F, Args...>), F, Args...> : std::true_type {
};

template <typename F, typename... Args>
using is_invocable = typename is_invocable_impl<void, F, Args...>::type;

// CPP 17 invoke implementation end

template <typename T, typename = void>
struct is_hashable : std::false_type {
};

template <typename T>
struct is_hashable<
    T,
    std::enable_if_t<std::is_same<std::size_t, decltype(declval<std::hash<T> const&>()(declval<T const&>()))>::value>>
  : std::true_type {
};

template <typename T, bool>
union optional_storage_impl;

template <typename T>
using optional_storage = optional_storage_impl<T, std::is_trivially_destructible<T>::value>;

template <typename T>
union optional_storage_impl<T, false> {

  optional_storage_impl(nullopt_t) : null_{} {}

  template <typename... Args>
  optional_storage_impl(Args&&... args) : value_{std::forward<Args>(args)...}
  {
  }

  ~optional_storage_impl() {}

  template <typename... Args>
  void emplace(Args&&... args)
  {
    new (&value_) T{std::forward<Args>(args)...};
  }

  T& get() & noexcept { return value_; }

  T const& get() const& noexcept { return value_; }

  T&& get() && noexcept { return (T &&)(value_); }

private:

  nullopt_t null_;
  T         value_;
};

template <typename T>
union optional_storage_impl<T, true> {

  constexpr optional_storage_impl(nullopt_t) : null_{} {}

  template <typename... Args>
  explicit constexpr optional_storage_impl(Args&&... args) : value_{std::forward<Args>(args)...}
  {
  }

  // default destructor
  // ~optional_storage_impl() = default;

  template <typename... Args>
  constexpr void emplace(Args&&... args)
  {
    new (&value_) T{std::forward<Args>(args)...};
  }

  constexpr T& get() & noexcept { return value_; }

  constexpr T const& get() const& noexcept { return value_; }

  constexpr T&& get() && noexcept { return (T &&)(value_); }

private:

  nullopt_t null_;
  T         value_;
};

template <typename T, bool>
class optional_base_impl;

template <typename T>
using optional_base = optional_base_impl<T, std::is_trivially_destructible<T>::value>;

template <typename T>
class optional_base_impl<T, false>
{

  using storage_type = optional_storage<T>;

public:

  optional_base_impl() noexcept = default;

  optional_base_impl(nullopt_t) noexcept : optional_base_impl{} {}

  template <typename... Args>
  explicit optional_base_impl(in_place_t, Args&&... args) : initialized_(true), storage_{std::forward<Args>(args)...}
  {
  }

  optional_base_impl(optional_base_impl const&) = default;
  optional_base_impl& operator=(optional_base_impl const&) = default;

  optional_base_impl(optional_base_impl&&) = default;
  optional_base_impl& operator=(optional_base_impl&&) = default;

  ~optional_base_impl()
  {
    if (initialized_)
      storage_.get().~T();
  }

protected:

  void reset() noexcept
  {
    if (initialized_)
      storage_.get().~T();
    initialized_ = false;
  }

  bool         initialized_ = false;
  storage_type storage_ = nullopt_t{};
};

template <typename T>
class optional_base_impl<T, true>
{

  using storage_type = optional_storage<T>;

public:

  constexpr optional_base_impl() noexcept = default;

  constexpr optional_base_impl(nullopt_t) noexcept : optional_base_impl{} {}

  template <typename... Args>
  constexpr explicit optional_base_impl(in_place_t, Args&&... args)
    : initialized_(true), storage_{std::forward<Args>(args)...}
  {
  }

  optional_base_impl(optional_base_impl const&) = default;
  optional_base_impl& operator=(optional_base_impl const&) = default;

  optional_base_impl(optional_base_impl&&) = default;
  optional_base_impl& operator=(optional_base_impl&&) = default;

  ~optional_base_impl() = default;

protected:

  constexpr void reset() noexcept { initialized_ = false; }

  bool         initialized_ = false;
  storage_type storage_ = nullopt_t{};
};

template <typename T, bool, bool>
struct optional_cc_base_impl;

template <typename T>
using optional_cc_base =
    optional_cc_base_impl<T, std::is_copy_constructible<T>::value, std::is_trivially_copy_constructible<T>::value>;

template <typename T, bool TriviallyCopyConstructible>
struct optional_cc_base_impl<T, false, TriviallyCopyConstructible> : public optional_base<T> {

  using optional_base<T>::optional_base;

  optional_cc_base_impl(optional_cc_base_impl const&) = delete;
  optional_cc_base_impl& operator=(optional_cc_base_impl const&) = default;

  optional_cc_base_impl(optional_cc_base_impl&&) = default;
  optional_cc_base_impl& operator=(optional_cc_base_impl&&) = default;
};

template <typename T>
struct optional_cc_base_impl<T, true, false> : public optional_base<T> {

  using optional_base<T>::optional_base;

  constexpr optional_cc_base_impl(optional_cc_base_impl const& other) noexcept(
      std::is_nothrow_copy_constructible<T>::value)
    : optional_base<T>{nullopt}
  {
    if ((this->initialized_ = other.initialized_))
      this->storage_.emplace(other.storage_.get());
  }
  optional_cc_base_impl& operator=(optional_cc_base_impl const&) = default;

  optional_cc_base_impl(optional_cc_base_impl&&) = default;
  optional_cc_base_impl& operator=(optional_cc_base_impl&&) = default;
};

template <typename T>
struct optional_cc_base_impl<T, true, true> : public optional_base<T> {
  using optional_base<T>::optional_base;

  optional_cc_base_impl(optional_cc_base_impl const&) = default;
  optional_cc_base_impl& operator=(optional_cc_base_impl const&) = default;

  optional_cc_base_impl(optional_cc_base_impl&&) = default;
  optional_cc_base_impl& operator=(optional_cc_base_impl&&) = default;
};

template <typename T, bool, bool>
struct optional_ca_base_impl;

template <typename T>
using optional_ca_base = optional_ca_base_impl<
    T,
    std::is_copy_assignable<T>::value && std::is_copy_constructible<T>::value,
    std::is_trivially_copy_constructible<T>::value && std::is_trivially_copy_assignable<T>::value &&
        std::is_trivially_destructible<T>::value>;

template <typename T, bool TriviallyCopyAssignable>
struct optional_ca_base_impl<T, false, TriviallyCopyAssignable> : public optional_cc_base<T> {
  using optional_cc_base<T>::optional_cc_base;

  optional_ca_base_impl(optional_ca_base_impl const&) = default;
  optional_ca_base_impl& operator=(optional_ca_base_impl const&) = delete;

  optional_ca_base_impl(optional_ca_base_impl&&) = default;
  optional_ca_base_impl& operator=(optional_ca_base_impl&&) = default;
};

template <typename T>
struct optional_ca_base_impl<T, true, false> : public optional_cc_base<T> {
  using optional_cc_base<T>::optional_cc_base;

  optional_ca_base_impl(optional_ca_base_impl const&) = default;
  constexpr optional_ca_base_impl& operator=(optional_ca_base_impl const& other) noexcept(
      std::is_nothrow_copy_constructible<T>::value&& std::is_nothrow_copy_assignable<T>::value)
  {
    if (this != &other) {
      if (this->initialized_) {
        if (other.initialized_)
          this->storage_.get() = other.storage_.get();
        else
          this->reset();
      } else if (other.initialized_) {
        this->storage_.emplace(other.storage_.get());
        this->initialized_ = true;
      }
    }
    return *this;
  };

  optional_ca_base_impl(optional_ca_base_impl&&) = default;
  optional_ca_base_impl& operator=(optional_ca_base_impl&&) = default;
};

template <typename T>
struct optional_ca_base_impl<T, true, true> : public optional_cc_base<T> {
  using optional_cc_base<T>::optional_cc_base;

  optional_ca_base_impl(optional_ca_base_impl const&) = default;
  optional_ca_base_impl& operator=(optional_ca_base_impl const&) = default;

  optional_ca_base_impl(optional_ca_base_impl&&) = default;
  optional_ca_base_impl& operator=(optional_ca_base_impl&&) = default;
};

template <typename T, bool, bool>
struct optional_mc_base_impl;

template <typename T>
using optional_mc_base =
    optional_mc_base_impl<T, std::is_move_constructible<T>::value, std::is_trivially_move_constructible<T>::value>;

template <typename T, bool TriviallyMoveConstructible>
struct optional_mc_base_impl<T, false, TriviallyMoveConstructible> : public optional_ca_base<T> {
  using optional_ca_base<T>::optional_ca_base;

  optional_mc_base_impl(optional_mc_base_impl const&) = default;
  optional_mc_base_impl& operator=(optional_mc_base_impl const&) = default;

  optional_mc_base_impl(optional_mc_base_impl&&) = delete;
  optional_mc_base_impl& operator=(optional_mc_base_impl&&) = default;
};

template <typename T>
struct optional_mc_base_impl<T, true, false> : public optional_ca_base<T> {
  using optional_ca_base<T>::optional_ca_base;

  optional_mc_base_impl(optional_mc_base_impl const& other) = default;
  optional_mc_base_impl& operator=(optional_mc_base_impl const&) = default;

  constexpr optional_mc_base_impl(optional_mc_base_impl&& other) noexcept(std::is_nothrow_move_constructible<T>::value)
    : optional_ca_base<T>{nullopt}
  {
    if ((this->initialized_ = other.initialized_))
      this->storage_.emplace(std::move(other.storage_).get());
  }
  optional_mc_base_impl& operator=(optional_mc_base_impl&&) = default;
};

template <typename T>
struct optional_mc_base_impl<T, true, true> : public optional_ca_base<T> {
  using optional_ca_base<T>::optional_ca_base;

  optional_mc_base_impl(optional_mc_base_impl const&) = default;
  optional_mc_base_impl& operator=(optional_mc_base_impl const&) = default;

  optional_mc_base_impl(optional_mc_base_impl&&) = default;
  optional_mc_base_impl& operator=(optional_mc_base_impl&&) = default;
};

template <typename T, bool, bool>
struct optional_ma_base_impl;

template <typename T>
using optional_ma_base = optional_ma_base_impl<
    T,
    std::is_move_assignable<T>::value && std::is_move_constructible<T>::value,
    std::is_trivially_move_constructible<T>::value && std::is_trivially_move_assignable<T>::value &&
        std::is_trivially_destructible<T>::value>;

template <typename T, bool TriviallyMoveAssignable>
struct optional_ma_base_impl<T, false, TriviallyMoveAssignable> : public optional_mc_base<T> {
  using optional_mc_base<T>::optional_mc_base;

  optional_ma_base_impl(optional_ma_base_impl const&) = default;
  optional_ma_base_impl& operator=(optional_ma_base_impl const&) = default;

  optional_ma_base_impl(optional_ma_base_impl&&) = default;
  optional_ma_base_impl& operator=(optional_ma_base_impl&&) = delete;
};

template <typename T>
struct optional_ma_base_impl<T, true, false> : public optional_mc_base<T> {
  using optional_mc_base<T>::optional_mc_base;

  optional_ma_base_impl(optional_ma_base_impl const& other) = default;
  optional_ma_base_impl& operator=(optional_ma_base_impl const&) = default;

  optional_ma_base_impl(optional_ma_base_impl&& other) = default;
  constexpr optional_ma_base_impl& operator=(optional_ma_base_impl&& other) noexcept(
      std::is_nothrow_move_constructible<T>::value&& std::is_nothrow_move_assignable<T>::value)
  {
    if (this != &other) {
      if (this->initialized_) {
        if (other.initialized_)
          this->storage_.get() = std::move(other.storage_).get();
        else
          this->reset();
      } else if (other.initialized_) {
        this->storage_.emplace(std::move(other.storage_).get());
        this->initialized_ = true;
      }
    }
    return *this;
  }
};

template <typename T>
struct optional_ma_base_impl<T, true, true> : public optional_mc_base<T> {
  using optional_mc_base<T>::optional_mc_base;

  optional_ma_base_impl(optional_ma_base_impl const&) = default;
  optional_ma_base_impl& operator=(optional_ma_base_impl const&) = default;

  optional_ma_base_impl(optional_ma_base_impl&&) = default;
  optional_ma_base_impl& operator=(optional_ma_base_impl&&) = default;
};

template <typename T, typename U>
using is_valid_constructor_conversion = typename m_all_of<
    m_not<std::is_constructible<T, optional<U>&>>,
    m_not<std::is_constructible<T, optional<U> const&>>,
    m_not<std::is_constructible<T, optional<U>&&>>,
    m_not<std::is_constructible<T, optional<U> const&&>>,
    m_not<std::is_convertible<T, optional<U>&>>,
    m_not<std::is_convertible<T, optional<U> const&>>,
    m_not<std::is_convertible<T, optional<U>&&>>,
    m_not<std::is_convertible<T, optional<U> const&&>>>::type;

template <typename T, typename U>
using is_valid_assignment_conversion = typename m_all_of<
    is_valid_constructor_conversion<T, U>,
    m_not<std::is_assignable<T&, optional<U>&>>,
    m_not<std::is_assignable<T&, optional<U> const&>>,
    m_not<std::is_assignable<T&, optional<U>&&>>,
    m_not<std::is_assignable<T&, optional<U> const&&>>>::type;

}  // namespace optional_ns

template <typename T>
class optional : public optional_ns::optional_ma_base<T>
{

  using base_type = optional_ns::optional_ma_base<T>;

public:

  constexpr optional() = default;

  constexpr optional(nullopt_t) noexcept : base_type{} {}

  optional(optional const&) = default;

  optional(optional&&) = default;

  template <
      typename U,
      typename = std::enable_if_t<
          std::is_constructible<T, U const&>::value && optional_ns::is_valid_constructor_conversion<T, U>::value>>
  constexpr optional(optional<U> const& other) noexcept(std::is_nothrow_constructible<T, U const&>::value) : optional{}
  {
    if ((this->initialized_ = other.initialized_))
      this->storage_.emplace(other.storage_.get());
  }

  template <
      typename U,
      typename = std::enable_if_t<
          std::is_constructible<T, U&&>::value && optional_ns::is_valid_constructor_conversion<T, U>::value>>
  constexpr optional(optional<U>&& other) noexcept(std::is_nothrow_constructible<T, U>::value) : optional{}
  {
    if ((this->initialized_ = other.initialized_))
      this->storage_.emplace(std::move(other.storage_).get());
  }

  template <typename... Args>
  constexpr explicit optional(in_place_t, Args&&... args) noexcept(std::is_nothrow_constructible<T, Args...>::value)
    : base_type{in_place, std::forward<Args>(args)...}
  {
  }

  template <
      typename U,
      typename... Args,
      typename = std::enable_if_t<std::is_constructible<T, std::initializer_list<U>&, Args&&...>::value>>
  constexpr explicit optional(in_place_t, std::initializer_list<U> ilist, Args&&... args) noexcept(
      std::is_nothrow_constructible<T, std::initializer_list<U>&, Args&&...>::value)
    : base_type{in_place, ilist, std::forward<Args>(args)...}
  {
  }

  template <
      typename U = T,
      typename = std::enable_if_t<
          std::is_constructible<T, U>::value &&
              !(std::is_same<std::decay_t<U>, in_place_t>::value || std::is_same<std::decay_t<U>, optional<T>>::value),
          void>>
  constexpr optional(U&& u) noexcept(std::is_nothrow_constructible<T, U>::value)
    : base_type{in_place, std::forward<U>(u)}
  {
  }

  ~optional() = default;

  constexpr optional& operator=(nullopt_t) noexcept { this->reset(); }

  optional& operator=(optional const&) = default;

  optional& operator=(optional&&) = default;

  template <
      typename U = T,
      typename = std::enable_if_t<optional_ns::m_all_of<
          optional_ns::m_not<std::is_same<std::decay_t<U>, optional<T>>>,
          std::is_constructible<T, U>,
          std::is_assignable<T&, U>,
          optional_ns::m_any_of<
              optional_ns::m_not<std::is_scalar<T>>,
              optional_ns::m_not<std::is_same<std::decay_t<U>, T>>>>::value>>
  constexpr optional&
  operator=(U&& value) noexcept(std::is_nothrow_constructible<T, U>::value&& std::is_nothrow_assignable<T&, U>::value)
  {
    if (this->initialized_) {
      this->storage_.get() = std::forward<U>(value);
    } else {
      this->storage_.emplace(std::forward<U>(value));
      this->initialized_ = true;
    }
    return *this;
  }

  template <
      typename U,
      typename = std::enable_if_t<
          std::is_constructible<T, U const&>::value && std::is_assignable<T&, U const&>::value &&
          optional_ns::is_valid_assignment_conversion<T, U>::value>>
  constexpr optional& operator=(optional<U> const& other) noexcept(
      std::is_nothrow_constructible<T, U const&>::value&& std::is_nothrow_assignable<T&, U const&>::value)
  {
    if (this->initialized_) {
      if (other.initialized_)
        this->storage_.get() = other.storage_.get();
      else
        this->reset();
    } else if (other.initialized_) {
      this->storage_.emplace(other.storage_.get());
      this->initialized_ = true;
    }
    return *this;
  }

  template <
      typename U,
      typename = std::enable_if_t<
          std::is_constructible<T, U>::value && std::is_assignable<T&, U>::value &&
          optional_ns::is_valid_assignment_conversion<T, U>::value>>
  constexpr optional& operator=(optional<U>&& other) noexcept(
      std::is_nothrow_constructible<T, U>::value&& std::is_nothrow_assignable<T&, U>::value)
  {
    if (this->initialized_) {
      if (other.initialized_)
        this->storage_.get() = std::move(other.storage_).get();
      else
        this->reset();
    } else if (other.initialized_) {
      this->storage_.emplace(std::move(other.storage_).get());
      this->initialized_ = true;
    }
    return *this;
  }

  constexpr T const* operator->() const noexcept { return &this->storage_.get(); }

  constexpr T* operator->() noexcept { return &this->storage_.get(); }

  constexpr T const& operator*() const& noexcept { return this->storage_.get(); }

  constexpr T& operator*() & noexcept { return this->storage_.get(); }

  constexpr T const&& operator*() const&& noexcept { return std::move(this->storage_).get(); }

  constexpr T&& operator*() && noexcept { return std::move(this->storage_).get(); }

  constexpr explicit operator bool() const noexcept { return this->initialized_; }

  constexpr bool has_value() const noexcept { return this->initialized_; }

  constexpr T& value() &
  {
    if (!this->initialized_)
      throw bad_optional_access{};
    return this->storage_.get();
  }

  constexpr T const& value() const&
  {
    if (!this->initialized_)
      throw bad_optional_access{};
    return this->storage_.get();
  }

  constexpr T&& value() &&
  {
    if (!this->initialized_)
      throw bad_optional_access{};
    return std::move(this->storage_).get();
  }

  constexpr T const&& value() const&&
  {
    if (!this->initialized_)
      throw bad_optional_access{};
    return std::move(this->storage_).get();
  }

  template <typename U>
  constexpr T value_or(U&& default_value) const&
  {
    return this->initialized_ ? **this : static_cast<T>(std::forward<U>(default_value));
  }

  template <typename U>
  constexpr T value_or(U&& default_value) &&
  {
    return this->initialized_ ? std::move(**this) : static_cast<T>(std::forward<U>(default_value));
  }

  template <typename F>
  constexpr auto
  and_then(F&& fn) & noexcept(noexcept(optional_ns::invoke(std::forward<F>(fn), optional_ns::declval<T&>())))
  {
    using ret_type = std::decay_t<optional_ns::invoke_result_t<F, T&>>;
    static_assert(optional_ns::is_optional<ret_type>::value, "and_then fn should return optional<U>");

    if (this->initialized_)
      return optional_ns::invoke(std::forward<F>(fn), **this);
    return ret_type{};
  }

  template <typename F>
  constexpr auto
  and_then(F&& fn) const& noexcept(noexcept(optional_ns::invoke(std::forward<F>(fn), optional_ns::declval<T const&>())))
  {
    using ret_type = std::decay_t<optional_ns::invoke_result_t<F, T const&>>;
    static_assert(optional_ns::is_optional<ret_type>::value, "and_then fn should return optional<U>");

    if (this->initialized_)
      return optional_ns::invoke(std::forward<F>(fn), **this);
    return ret_type{};
  }

  template <typename F>
  constexpr auto
  and_then(F&& fn) && noexcept(noexcept(optional_ns::invoke(std::forward<F>(fn), optional_ns::declval<T>())))
  {
    using ret_type = std::decay_t<optional_ns::invoke_result_t<F, T>>;
    static_assert(optional_ns::is_optional<ret_type>::value, "and_then fn should return optional<U>");

    if (this->initialized_)
      return optional_ns::invoke(std::forward<F>(fn), *std::move(*this));
    return ret_type{};
  }

  template <typename F>
  constexpr auto and_then(F&& fn) const&& noexcept(
      noexcept(optional_ns::invoke(std::forward<F>(fn), optional_ns::declval<T const&&>())))
  {
    using ret_type = std::decay_t<optional_ns::invoke_result_t<F, T const&&>>;
    static_assert(optional_ns::is_optional<ret_type>::value, "and_then fn should return optional<U>");

    if (this->initialized_)
      return optional_ns::invoke(std::forward<F>(fn), *std::move(*this));
    return ret_type{};
  }

  template <typename F>
  constexpr auto
  transform(F&& fn) & noexcept(noexcept(optional_ns::invoke(std::forward<F>(fn), optional_ns::declval<T&>())))
  {
    using ret_type = std::decay_t<optional_ns::invoke_result_t<F, T&>>;
    using _ = void(decltype(ret_type(optional_ns::invoke(std::forward<F>(fn), **this))));

    if (this->initialized_)
      return optional<ret_type>{optional_ns::invoke(std::forward<F>(fn), **this)};
    return optional<ret_type>{};
  }

  template <typename F>
  constexpr auto transform(F&& fn) const& noexcept(
      noexcept(optional_ns::invoke(std::forward<F>(fn), optional_ns::declval<T const&>())))
  {
    using ret_type = std::decay_t<optional_ns::invoke_result_t<F, T const&>>;
    using _ = void(decltype(ret_type(optional_ns::invoke(std::forward<F>(fn), **this))));

    if (this->initialized_)
      return optional<ret_type>{optional_ns::invoke(std::forward<F>(fn), **this)};
    return optional<ret_type>{};
  }

  template <typename F>
  constexpr auto
  transform(F&& fn) && noexcept(noexcept(optional_ns::invoke(std::forward<F>(fn), optional_ns::declval<T>())))
  {
    using ret_type = std::decay_t<optional_ns::invoke_result_t<F, T>>;
    using _ = void(decltype(ret_type(optional_ns::invoke(std::forward<F>(fn), *std::move(*this)))));

    if (this->initialized_)
      return optional<ret_type>{optional_ns::invoke(std::forward<F>(fn), *std::move(*this))};
    return optional<ret_type>{};
  }

  template <typename F>
  constexpr auto transform(F&& fn) const&& noexcept(
      noexcept(optional_ns::invoke(std::forward<F>(fn), optional_ns::declval<T const&&>())))
  {
    using ret_type = std::decay_t<optional_ns::invoke_result_t<F, T const&&>>;
    using _ = void(decltype(ret_type(optional_ns::invoke(std::forward<F>(fn), *std::move(*this)))));

    if (this->initialized_)
      return optional<ret_type>{optional_ns::invoke(std::forward<F>(fn), *std::move(*this))};
    return optional<ret_type>{};
  }

  template <
      typename F,
      typename =
          std::enable_if_t<optional_ns::m_all_of<std::is_copy_constructible<T>, optional_ns::is_invocable<F>>::value>>
  constexpr auto or_else(F&& fn) const& noexcept(
      std::is_nothrow_copy_constructible<T>::value&& noexcept(optional_ns::invoke(std::forward<F>(fn))))
  {
    static_assert(
        std::is_same<std::decay_t<optional_ns::invoke_result_t<F>>, optional<T>>{},
        "or_else fn should return optional<T>");
    return this->initialized_ ? *this : optional_ns::invoke(std::forward<F>(fn));
  }

  template <
      typename F,
      typename =
          std::enable_if_t<optional_ns::m_all_of<std::is_move_constructible<T>, optional_ns::is_invocable<F>>::value>>
  constexpr auto or_else(F&& fn) && noexcept(
      std::is_nothrow_move_constructible<T>::value&& noexcept(optional_ns::invoke(std::forward<F>(fn))))
  {
    static_assert(
        std::is_same<std::decay_t<optional_ns::invoke_result_t<F>>, optional<T>>{},
        "or_else fn should return optional<T>");
    return this->initialized_ ? std::move(*this) : std::forward<F>(fn)();
  }

  constexpr void swap(optional& other) noexcept(
      std::is_nothrow_move_constructible<T>::value&& optional_ns::is_nothrow_swappable<T>::value)
  {
    static_assert(std::is_move_constructible<T>::value, "swap requires T to be move constructible");
    using std::swap;
    switch (has_value() + other.has_value()) {
      case 2: return swap(**this, *other);
      case 1:
        if (has_value()) {
          other.storage_.emplace(std::move(**this));
          reset();
        } else {
          this->storage_.emplace(std::move(*other));
          other.reset();
        }
      default: return;
    }
  }

  using base_type::reset;

  template <typename... Args>
  constexpr T& emplace(Args&&... args) noexcept(std::is_nothrow_constructible<T, Args...>::value)
  {
    reset();
    this->storage_.emplace(std::forward<Args>(args)...);
    this->initialized_ = true;
    return **this;
  }

  template <
      typename U,
      typename... Args,
      typename = std::enable_if_t<std::is_constructible<T, std::initializer_list<U>&, Args&&...>::value>>
  constexpr T& emplace(std::initializer_list<U> ilist, Args&&... args) noexcept(
      std::is_nothrow_constructible<T, std::initializer_list<U>&, Args...>::value)
  {
    reset();
    this->storage_.emplace(ilist, std::forward<Args>(args)...);
    this->initialized_ = true;
    return **this;
  }
};

template <typename T, typename U>
inline constexpr bool operator==(optional<T> const& lhs, optional<U> const& rhs)
{
  if (bool(lhs) != bool(rhs))
    return false;
  return lhs ? *lhs == *rhs : true;
}

template <typename T, typename U>
inline constexpr bool operator!=(optional<T> const& lhs, optional<U> const& rhs)
{
  if (bool(lhs) != bool(rhs))
    return true;
  return lhs ? *lhs != *rhs : false;
}

template <typename T, typename U>
inline constexpr bool operator<(optional<T> const& lhs, optional<U> const& rhs)
{
  if (!bool(rhs))
    return false;
  if (!bool(lhs))
    return true;
  return *lhs < *rhs;
}

template <typename T, typename U>
inline constexpr bool operator<=(optional<T> const& lhs, optional<U> const& rhs)
{
  if (!bool(lhs))
    return true;
  if (!bool(rhs))
    return false;
  return *lhs <= *rhs;
}

template <typename T, typename U>
inline constexpr bool operator>(optional<T> const& lhs, optional<U> const& rhs)
{
  if (!bool(lhs))
    return false;
  if (!bool(rhs))
    return true;
  return *lhs > *rhs;
}

template <typename T, typename U>
inline constexpr bool operator>=(optional<T> const& lhs, optional<U> const& rhs)
{
  if (!bool(lhs))
    return true;
  if (!bool(rhs))
    return false;
  return *lhs >= *rhs;
}

template <typename T>
inline constexpr bool operator==(optional<T> const& lhs, nullopt_t) noexcept
{
  return !lhs;
}

template <typename T>
inline constexpr bool operator==(nullopt_t, optional<T> const& rhs) noexcept
{
  return !rhs;
}

template <typename T>
inline constexpr bool operator!=(optional<T> const& lhs, nullopt_t) noexcept
{
  return bool(lhs);
}

template <typename T>
inline constexpr bool operator!=(nullopt_t, optional<T> const& rhs) noexcept
{
  return bool(rhs);
}

template <typename T>
inline constexpr bool operator<(optional<T> const& lhs, nullopt_t) noexcept
{
  return false;
}

template <typename T>
inline constexpr bool operator<(nullopt_t, optional<T> const& rhs) noexcept
{
  return bool(rhs);
}

template <typename T>
inline constexpr bool operator<=(optional<T> const& lhs, nullopt_t) noexcept
{
  return !lhs;
}

template <typename T>
inline constexpr bool operator<=(nullopt_t, optional<T> const& rhs) noexcept
{
  return true;
}

template <typename T>
inline constexpr bool operator>(optional<T> const& lhs, nullopt_t) noexcept
{
  return bool(lhs);
}

template <typename T>
inline constexpr bool operator>(nullopt_t, optional<T> const& rhs) noexcept
{
  return false;
}

template <typename T>
inline constexpr bool operator>=(optional<T> const& lhs, nullopt_t) noexcept
{
  return true;
}

template <typename T>
inline constexpr bool operator>=(nullopt_t, optional<T> const& rhs) noexcept
{
  return !rhs;
}

template <typename T, typename U>
inline constexpr bool operator==(optional<T> const& lhs, U const& rhs)
{
  return bool(lhs) ? *lhs == rhs : false;
}

template <typename T, typename U>
inline constexpr bool operator==(U const& lhs, optional<T> const& rhs)
{
  return bool(rhs) ? lhs == *rhs : false;
}

template <typename T, typename U>
inline constexpr bool operator!=(optional<T> const& lhs, U const& rhs)
{
  return bool(lhs) ? *lhs != rhs : true;
}

template <typename T, typename U>
inline constexpr bool operator!=(U const& lhs, optional<T> const& rhs)
{
  return bool(rhs) ? lhs != *rhs : true;
}

template <typename T, typename U>
inline constexpr bool operator<(optional<T> const& lhs, U const& rhs)
{
  return bool(lhs) ? *lhs < rhs : true;
}

template <typename T, typename U>
inline constexpr bool operator<(U const& lhs, optional<T> const& rhs)
{
  return bool(rhs) ? lhs < *rhs : false;
}

template <typename T, typename U>
inline constexpr bool operator<=(optional<T> const& lhs, U const& rhs)
{
  return bool(lhs) ? *lhs <= rhs : true;
}

template <typename T, typename U>
inline constexpr bool operator<=(U const& lhs, optional<T> const& rhs)
{
  return bool(rhs) ? lhs <= *rhs : false;
}

template <typename T, typename U>
inline constexpr bool operator>(optional<T> const& lhs, U const& rhs)
{
  return bool(lhs) ? *lhs > rhs : false;
}

template <typename T, typename U>
inline constexpr bool operator>(U const& lhs, optional<T> const& rhs)
{
  return bool(rhs) ? lhs > *rhs : true;
}

template <typename T, typename U>
inline constexpr bool operator>=(optional<T> const& lhs, U const& rhs)
{
  return bool(lhs) ? *lhs >= rhs : false;
}

template <typename T, typename U>
inline constexpr bool operator>=(U const& lhs, optional<T> const& rhs)
{
  return bool(rhs) ? lhs >= *rhs : true;
}

namespace optional_ns
{

template <typename T, bool>
struct hash_impl;

template <typename T>
using hash = hash_impl<T, is_hashable<T>::value>;

template <typename T>
struct hash_impl<T, true> {
  constexpr std::size_t operator()(optional<T> const& o) const
  {
    constexpr std::size_t magic_empty_hash = std::size_t(-4777);
    return o ? std::hash<T>{}(*o) : magic_empty_hash;
  }
};

template <typename T>
struct hash_impl<T, false> {
};

}  // namespace optional_ns

}  // namespace simple

template <typename T>
struct std::hash<simple::optional<T>> : simple::optional_ns::hash<T> {
};

#endif  // SIMPLE_OPTIONAL_HPP
