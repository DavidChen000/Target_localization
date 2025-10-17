"""
Some useful functions to manipulate bytes
Author: Mohamed Abdelkader
Contact: mohamedashraf123@gmail.com
"""

def toHex(intval, nbits):
    """
    Converts an integer to hexdecimal.
    Useful for negative integers where hex() doesn't work as expected

    Params
    --
    intaval: [int] Integer number
    nbits: [int] Number of bits

    Returns
    --
    String of the hexdecimal value
    """
    h = format((intval + (1 << nbits)) % (1 << nbits),'x')
    if len(h)==1:
        h="0"+h
    return h

def toInt(hexval):
    """
    Converts hexidecimal value to an integer number, which can be negative
    Ref: https://www.delftstack.com/howto/python/python-hex-to-int/

    Params
    --
    hexval: [string] String of the hex value
    """
    bits = 16
    val = int(hexval, bits)
    if val & (1 << (bits-1)):
        val -= 1 << bits
    return val

def toTwoBits(data):
    """
    Determine if a string is a 4-digit number, and if not, pad it with zeros at the front to make it four digits.
    Then swap the high and low order bytes

    Params
    --
    data: [string] 
    """
    if len(data) == 4:
        high_byte = data[:2]
        low_byte = data[2:]
        return low_byte+high_byte
    else:
        data = "0"*(4-len(data)) + data
        high_byte = data[:2]
        low_byte = data[2:]
        return low_byte+high_byte
