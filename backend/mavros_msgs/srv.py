class SetMode:
    class Request:
        custom_mode: str = ''
class CommandBool:
    class Request:
        value: bool = True
class CommandLong:
    class Request:
        broadcast: bool = False
        command: int = 0
        confirmation: int = 0
        param1: float = 0.0
        param2: float = 0.0
        param3: float = 0.0
        param4: float = 0.0
        param5: float = 0.0
        param6: float = 0.0
        param7: float = 0.0
    class Response:
        success: bool = False
        result: int = 0
