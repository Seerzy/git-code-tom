#include <iostream>

class MyClass {
public:
    static int staticVariable; // A static variable shared by all instances.
    int nonStaticVariable;    // A non-static variable specific to each instance.

    MyClass(int value) : nonStaticVariable(value) {}
    void test(){
        std::cout << staticVariable << std::endl;
    }
};

int MyClass::staticVariable = 0; // Initialization of the static variable

int main() {
    MyClass obj1(42);
    MyClass obj2(123);

    obj1.staticVariable = 1; // Modifies the static variable for all instances
    // obj1.nonStaticVariable = 100; // Modifies only for obj1

    // obj2.nonStaticVariable = 200; // Modifies only for obj2
    obj1.test();
    std::cout << MyClass::staticVariable << std::endl; // Accessing the static variable
    std::cout << obj1.nonStaticVariable << std::endl; // Accessing non-static for obj1
    std::cout << obj2.nonStaticVariable << std::endl; // Accessing non-static for obj2

    return 0;
}
