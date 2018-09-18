
namespace dev {
inline namespace v1 {
class Abstract {
 public:
  virtual int setup() = 0;
};
}  // namespace v1
}  // namespace dev

namespace dev {
inline namespace dv1 {
class Derived : public Abstract {
 public:
  virtual int setup() override;
};
}  // namespace dv1
}  // namespace dev

namespace dev {
namespace dv1 {
int Derived::setup() { return 0; }
}  // namespace dv1
}  // namespace dev

dev::Derived d;

//#define CPP
#ifndef CPP

void setup() {
  Serial.begin(9600);
  Serial.println(d.setup());
}

void loop() {}

#else

#include <iostream>

int main(int argc, char const **argv) {
  std::cout << d.setup() << std::endl;
  return 0;
}

#endif