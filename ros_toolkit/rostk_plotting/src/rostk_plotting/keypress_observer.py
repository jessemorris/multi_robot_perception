#! /usr/bin/env python


from pynput.keyboard import Listener, Key

from rostk_plotting.keypress_listener import KeypressListener
from rostk_pyutils.observer import Observer





class KeypressObserver():

    #key should be the key press
    #value will be a list of KeypressListeners whi are listening for that keypress and should
    # be notified
    keypress_listeners = {}

    def __init__(self):
        self.keyboard_listener = Listener(on_press=KeypressObserver.press_callback)
        self.keyboard_listener.start()

    def attach(self, subject):
        assert isinstance(KeypressListener, subject)

        if subject.event_key in KeypressObserver.keypress_listeners:
            KeypressObserver.keypress_listeners[subject.event_key].append(subject)
        else:
            KeypressObserver.keypress_listeners[subject.event_key] = [subject]






    @staticmethod
    def press_callback(key):
        print("{} was pressed".format(key.char))