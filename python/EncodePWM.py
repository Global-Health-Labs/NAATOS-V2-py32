# distribute_bits
#
# create a 256 bit wide bitfield

def distribute_bits(n: int) -> int:
    """
    Distributes N ones evenly across a 256-bit wide bitfield.

    Parameters:
    - n (int): The number of ones to distribute (0-256).

    Returns:
    - int: A 256-bit number with N ones distributed across the bitfield.
    """
    if not (0 <= n <= 256):
        raise ValueError("Input must be between 0 and 256 (inclusive).")

    bitfield = 0
    if n == 0:
        return bitfield

    # Calculate the spacing between the ones
    step = 256 // n  # Integer division to determine spacing
    for i in range(n):
        bitfield |= (1 << (i * step))

    return bitfield

bitfield_size = 256
input_bits = 8

def distribute_bits2(n: int) -> int:
    bitfield = 0

    if n == 0:
        return bitfield

    for i in range(input_bits):
        bit_pos = input_bits - i - 1
        if (n>>bit_pos) & 1:
            base_val = bitfield_size // (2**bit_pos)
            sub_bitfield = base_val
            shift = base_val
            for j in range(1,bit_pos+1):
                sub = (sub_bitfield << shift)
                sub_bitfield |= sub
                #print(f"\tj: {j} sub: {bin(sub)} sub_bitfield: {bin(sub_bitfield)} shift {shift}")
                shift *= 2
            print(f"{bit_pos} base_val: {bin(base_val)} sub_bitfield: {bin(sub_bitfield)}")
            bitfield |= sub_bitfield
        else:
            print(f"skipping bit_pos {bit_pos}")

    return bitfield

def distribute_bits3(n: int) -> int:
    bitfield = 0

    if n == 0:
        return bitfield

    data_pos = 1
    for i in range(input_bits):
        bit_pos = input_bits - i - 1
        if (n>>bit_pos) & 1:
            base_val = bitfield_size // (2**bit_pos)
            sub_bitfield = (1 << data_pos)
            shift = base_val
            for j in range(1,bit_pos+1):
                sub_bitfield |= (sub_bitfield << shift)
                #print(f"\tj: {j} sub_bitfield: {bin(sub_bitfield)} shift {shift}")
                shift *= 2
            print(f"{bit_pos} data_pos: {data_pos} sub_bitfield: {bin(sub_bitfield)}")
            bitfield |= sub_bitfield
        #else:
            #print(f"skipping bit_pos {bit_pos}")
        data_pos *= 2

    return bitfield

def distribute_bit(tick: int, pwm_val: int) -> int:
    step = 256 // pwm_val  # Integer division to determine spacing
    for i in range(pwm_val):
        position = i * step
        if tick == position: return 1
    return 0


if __name__ == "__main__":
    pwm = 255
    bitfield = distribute_bits(pwm)
    print(f"Distributed value: {bitfield:#066x}")  # Print as 256-bit hex value

    bitcount = 0
    for i in range(256):
        if bitfield & 0x1: bitcount += 1
        bitfield = bitfield >> 1
    print(f"bitcount: {bitcount} ")

    bitcount = 0
    for i in range(256):
        val = distribute_bit(i, pwm)
        print(f"{val}", end="")
        if val == 1: bitcount += 1

    print(f"\nbitcount: {bitcount} ")

    bitfield = distribute_bits3(pwm)
    print(f"distribute_bits3: {bitfield:#08x}")
    print(f"{bin(bitfield)}")
    bitcount = 0
    for i in range(bitfield_size):
        if bitfield & 0x1: bitcount += 1
        bitfield = bitfield >> 1
    print(f"bitcount: {bitcount} ")

