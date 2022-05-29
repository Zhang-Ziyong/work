#include <memory>
#include <iostream>

class TestInstance {
public:
    ~TestInstance() {
        std::cout << "Destory  test ???? "<< std::endl;
    }

    static std::shared_ptr<TestInstance> getPtrInstance() {
        if (!ptr_ins_.get()) {
            ptr_ins_.reset(new TestInstance());
        }
        return ptr_ins_;
    }

    void show() {
        std::cout << "Hello Test!" << std::endl;
    }

private:
    TestInstance() {
        std::cout << "Build Test!!!!" << std::endl;
    };

    static std::shared_ptr<TestInstance> ptr_ins_;
};

std::shared_ptr<TestInstance> TestInstance::ptr_ins_ = nullptr;

int main() {
    TestInstance::getPtrInstance()->show();
    TestInstance::getPtrInstance()->show();

    TestInstance::getPtrInstance()->show();
    TestInstance::getPtrInstance()->show();

    return 1;
}