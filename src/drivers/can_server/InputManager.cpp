
#include <stdio.h>
#include <stdlib.h>
#include <tgf.h>
#include <tgfclient.h>

#include "InputManager.h"


static int _onSKeyAction(int key, int modifier, int state);
static int _onKeyAction(unsigned char key, int modifier, int state);

InputManager* InputManager::getInstance() {
    static InputManager sin;
    return &sin;
}

InputManager::InputManager() {
    this->attached = false;
	memset(this->currentKey, 0, sizeof(this->currentKey));
	memset(this->currentSKey, 0, sizeof(this->currentSKey));
}

void InputManager::attach() {
    if (!this->attached) {
		GfuiKeyEventRegisterCurrent(_onKeyAction);
		GfuiSKeyEventRegisterCurrent(_onSKeyAction);
        this->attached = true;
    }
}

void InputManager::detach() {
    if (this->attached) {
		GfuiKeyEventRegisterCurrent(NULL);
		GfuiSKeyEventRegisterCurrent(NULL);
        this->attached = true;
    }
}

int InputManager::onKeyAction(unsigned char key, int modifier, int state) {
	this->currentKey[key] = state;
	return 0;
}

int InputManager::onSKeyAction(int key, int modifier, int state) {
	this->currentSKey[key] = state;
	return 0;
}

static int _onKeyAction(unsigned char key, int modifier, int state) {
	return InputManager::getInstance()->onKeyAction(key, modifier, state);
}

static int _onSKeyAction(int key, int modifier, int state) {
	return InputManager::getInstance()->onSKeyAction(key, modifier, state);
}

bool InputManager::isKeyDown(uint16_t key) {
    if (key & 0x0100) {
        return this->currentSKey[key & 0xff] == GFUI_KEY_DOWN;
    } else {
        return this->currentKey[key & 0xff] == GFUI_KEY_DOWN;
    }
}
