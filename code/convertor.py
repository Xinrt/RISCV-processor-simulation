def int_to_bitstring(bit: int) -> str:

    if not isinstance(bit, int):
        if isinstance(bit, str):
            while len(bit) < 32:
                bit = '0' + bit
            return bit
        else:
            raise Exception('The input is neither a int nor a string')
    if bit < 0:
        reverse_bit = -bit - 1
        reverse_bitstring = bin(reverse_bit)[2:]
        bitstring = ''
        for bit in reverse_bitstring:
            if bit == '1':
                bitstring += '0'
            else:
                bitstring += '1'

        if len(bitstring) > 32:
            bitstring = bitstring[-32:]
        while len(bitstring) < 32:
            bitstring = '1' + bitstring
    else:
        bitstring = bin(bit)[2:]
        if len(bitstring) > 32:
            bitstring = bitstring[-32:]

        while len(bitstring) < 32:
            bitstring = '0' + bitstring
    return bitstring


def bitstring_to_int(bitstring):
    uint = int(bitstring, 2)
    bitlength = len(bitstring)
    if (uint & (1 << (bitlength - 1))) != 0:
        uint = uint - (1 << bitlength)
    return uint