
class KeypressListener():

    def __init__(self, key_event_string):

        #the key to respond too
        #should be string
        self._event_key = key_event_string

    @property
    def event_key(self):
        return self._event_key

    def triggger(self, key):
        raise NotImplementedError