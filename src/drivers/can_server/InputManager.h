class InputManager {
private:
    bool attached;
    int currentKey[256];
    int currentSKey[256];
public:
    static InputManager *getInstance();
    InputManager();
    void attach();
    void detach();
    int onKeyAction(unsigned char key, int modifier, int state);
    int onSKeyAction(int key, int modifier, int state);
    bool isKeyDown(uint16_t key);
};
