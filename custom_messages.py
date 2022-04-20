import imp
from typing import Tuple
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class CustomString(genpy.Message):
    '''Кастомное сообщение для ноды'''

    __slots__ = ['cs_ms']
    _slot_types = ['string']



class ComandMessage(genpy.Message):
    '''Кастомное сообщение для ноды'''


    _md5sum = "4a842b65f413084dc2b10fb484ea7f17"
    _type = "custom_messages/ComandMessage"
    _has_header = False  # flag to mark the presence of a Header object
    _full_text = "Custom message"

    __slots__ = ['START','END','command_type', 'name', 'step']
    _slot_types = ['string','string', 'string', 'string', 'float64']

    def __init__(self, *args, **kwds):
        """
        Constructor. Any message fields that are implicitly/explicitly
        set to None will be assigned a default value. The recommend
        use is keyword arguments as this is more robust to future message
        changes.  You cannot mix in-order arguments and keyword arguments.

        The available fields are:
           START, END, command type, data name, step of trace

        :param args: complete set of field values, in .msg order
        :param kwds: use keyword arguments corresponding to message field names
        to set specific fields.
        """
        if args or kwds:
            super(ComandMessage, self).__init__(*args, **kwds)
            # message fields cannot be None, assign default values for those that are
            if self.START is None:
                self.START = 'START'
            if self.END is None:
                self.END = 'END'
            if self.command_type is None:
                self.command_type = 'END'
            if self.name is None:
                self.name = ''
            if self.step is None:
                self.step = 0.1

        else:
            self.START = 'START'
            self.END = 'START'
            self.command_type = 'END'
            self.name = ''
            self.step = 0.1

    def _get_types(self):
        """
        internal API method
        """
        return self._slot_types

    def serialize(self, buff):
        """
        serialize message into buffer
        :param buff: buffer, ``StringIO``
        """
        try:
            self._write_str(buff, self.START)
            self._write_str(buff, self.END)
            self._write_str(buff, self.command_type)
            self._write_str(buff, self.name)
            
            _x = self
            buff.write(_get_struct_d().pack(_x.step))
        except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
        except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

    def deserialize(self, s):
        """
        unpack serialized message in str into this message instance
        :param str: byte array of serialized message, ``str``
        """
        # self.START = s.decode('utf-8', 'rosmsg')
        #self.START = type(s)

        if python3:
            codecs.lookup_error("rosmsg").msg_type = self._type
        try:
            offset = 0

            (self.START, offset) = self.read_str(s, offset)
            
            (self.END, offset) = self.read_str(s, offset)

            (self.command_type, offset) = self.read_str(s, offset)

            (self.name, offset) = self.read_str(s, offset)

            self.step = offset
            
            end = offset
            start = end
            end += 8
            (self.step,) = _get_struct_d().unpack(s[start:end])

            return self
        except struct.error as e:
            raise genpy.DeserializationError(e)  # most likely buffer underfill


    def serialize_numpy(self, buff, numpy):
        """
        serialize message with numpy array types into buffer
        :param buff: buffer, ``StringIO``
        :param numpy: numpy python module
        """
        self.serialize(buff)


    @staticmethod
    def _write_str(buff, _x):
        length = len(_x)
        if python3 or type(_x) == unicode:
            _x = _x.encode('utf-8')
            length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
    

    def read_str(buff, s, start: int) -> Tuple[str, int]:
        '''Возвращяет строку и смещение которое было считано'''
        
        end = start
        end += 4
        (length,) = _struct_I.unpack(s[start:end])
        start = end
        end += length
        if python3:
            return (s[start:end].decode('utf-8', 'rosmsg'), end)
        else:
            return (s[start:end], end)


    def deserialize_numpy(self, s, numpy):
        """
        unpack serialized message in str into this message instance using numpy for array types
        :param str: byte array of serialized message, ``str``
        :param numpy: numpy python module
        """

        self.deserialize(s)


_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_d = None
def _get_struct_d():
    global _struct_d
    if _struct_d is None:
        _struct_d = struct.Struct("<d")
    return _struct_d
