class DevelopmentConfig():
    TESTING = False
    WTF_CSRF_ENABLED = False
    SECRET_KEY = 'secret'


class TestingConfig():
    TESTING = True
    WTF_CSRF_ENABLED = False
    PRESERVE_CONTEXT_ON_EXCEPTION = False
