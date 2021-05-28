"""
    Input file must be of the form:
    length code                     value char
        4 0000                      0      'r'
        4 0001                      1      's'
        3 001                       1      'e'
        4 0100                      4      'h'
        6 010100                   20      'b'
        6 010101                   21      ','

    Remove any labels before use and only use top half of default dictionary
"""


def main():
    with open("default_dict", "r") as file:
        lines = file.readlines()

    encoding = [0 for i in range(128)]

    for line in lines:
        bit_code_val_symbol = line.split()
        try:
            length, code, value, symbol = bit_code_val_symbol
            i_code = int(code, 2)
            i_length = int(length)
            if symbol == "'SPACE'":
                symbol = "' '"
            i_symbol = ord(symbol[1:-1])
            b_code = (0 ^ i_code) << 17
            b_length = (0 ^ i_length) << 8
            b_symbol = (0 ^ i_symbol)
            new_encoding = b_code | b_length | b_symbol
            encoding[i_symbol] = new_encoding
        except:
            continue

    for encd in encoding:
        char = chr(0xFF & encd)
        length = (encd >> 8) & 0xFF
        code = (encd >> 17)
        print("length: {l}\tchar: {ch}\t\tcode: {c}".format(l=length,c=bin(code)[2:].zfill(length), ch=char))

    with open("dict_output", "w") as f:
        print("[", file=f, end="")
        for i in range(127):
            print("{code}, ".format(code=encoding[i]), file=f, end="")
        print("{code}, ".format(code=encoding[127]), file=f, end="")
        print("]", file=f)



main()