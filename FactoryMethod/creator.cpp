//자료 출처 https://refactoring.guru/ko/design-patterns
#include <string>
using namespace std;

class Factory {
 public:
  virtual ~Factory() {}
  virtual std::string Function() const = 0;
};



class Product1 : public Factory {
 public:
  std::string Function() const override {
    return "Test 1 Object Product1";
  }
};
class Product2 : public Factory {
 public:
  std::string Function() const override {
    return "Test 1 Object Product2";
  }
};




class Creator {
 public:
  virtual ~Creator(){};
  virtual Factory* FactoryMethod() const = 0;
  std::string OtherFunction() const {
    Factory* product = this->FactoryMethod();
    std::string result = product->Function();
    delete product;
    return result;
  }
};


class Creator1 : public Creator {
 public:
  Factory* FactoryMethod() const override {
    return new Product1();
  }
};

class Creator2 : public Creator {
 public:
  Factory* FactoryMethod() const override {
    return new Product2();
  }
};

void ClientCode(const Creator& creator) {
  std::cout << creator.OtherFunction() << std::endl;
}


int main() {
  Creator* creator = new Creator1();
  ClientCode(*creator);
  Creator* creator2 = new Creator2();
  ClientCode(*creator2);

  delete creator;
  delete creator2;
  return 0;
}