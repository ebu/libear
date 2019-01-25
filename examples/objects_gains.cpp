#include <iomanip>
#include <iostream>
#include "ear/ear.hpp"

using namespace ear;

int main(int argc, char **argv) {
  // make the gain calculator
  Layout layout = getLayout("0+5+0");
  GainCalculatorObjects gc(layout);

  // make the input data; just left of centre
  ObjectsTypeMetadata otm;
  otm.position = PolarPosition(10.0f, 0.0f, 1.0f);

  // calculate the direct and diffuse gains
  std::vector<float> directGains(layout.channels().size());
  std::vector<float> diffuseGains(layout.channels().size());
  gc.calculate(otm, directGains, diffuseGains);

  // print the output
  auto fmt = std::setw(10);
  std::cout << std::setprecision(4);

  std::cout << fmt << "channel"  //
            << fmt << "direct"  //
            << fmt << "diffuse" << std::endl;
  for (size_t i = 0; i < layout.channels().size(); i++)
    std::cout << fmt << layout.channels()[i].name()  //
              << fmt << directGains[i]  //
              << fmt << diffuseGains[i] << std::endl;

  return 0;
}
