#!/usr/bin/env python3

# This script generates the built-in rom of RCPU


with open("build/rom.hex") as infile:
    # Format of this file: newline delimited, hex-encoded 16-bit values
    # For example:
    # 1234
    # ABCD
    # 589A
    code = [int(line, 16) for line in infile.read().split()]
    if len(code) != 2**12:
        raise ValueError(f"Expected {2**12} lines, got {len(code)}")

# The format of the INIT_x parameters:
# the first 16-bit word will end up last in the first parameter, so if
# initial mem[0] = 255;
# corresponds to
# .INIT_0(256'hxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx00ff),
# the 16-th 16-bit word will up as the last part of INIT_1

template = """
wire {readwire};
SB_RAM256x16 #(
    .INIT_0(256'h{init[0]}),
    .INIT_1(256'h{init[1]}),
    .INIT_2(256'h{init[2]}),
    .INIT_3(256'h{init[3]}),
    .INIT_4(256'h{init[4]}),
    .INIT_5(256'h{init[5]}),
    .INIT_6(256'h{init[6]}),
    .INIT_7(256'h{init[7]}),
    .INIT_8(256'h{init[8]}),
    .INIT_9(256'h{init[9]}),
    .INIT_A(256'h{init[10]}),
    .INIT_B(256'h{init[11]}),
    .INIT_C(256'h{init[12]}),
    .INIT_D(256'h{init[13]}),
    .INIT_E(256'h{init[14]}),
    .INIT_F(256'h{init[15]})
) _ram{memorymodule} (
    .RDATA({readwire}),
    .RADDR({read_addr}),
    .RCLK(clk), .RCLKE(1'b1), .RE({read_enable}),
    .WCLK(clk), .WCLKE(1'b1), .WE({write_enable}),
    .WADDR({write_addr}),
    .MASK(16'h0000), .WDATA({write_data}));
"""


def generate_proposition(memorymodule, address_name, unlock_name):
    '''Generates the logic propostion for the read_enable and write_enable'''
    propositions = [f'{unlock_name}']
    for idx, i in enumerate([8, 9, 10, 11]):
        if (1 << idx) & memorymodule:
            propositions.append(f'{address_name}[{i}]')
        else:
            propositions.append(f'!{address_name}[{i}]')
    return ' & '.join(propositions)


def generate_module(code, memorymodule, ramfile):
    # part of the rom that will be included in this memory module
    part = code[memorymodule * 2**8:(memorymodule+1) * 2**8]
    initvalues = {}
    for i in range(16):
        values = part[i * 16:(i + 1) * 16]
        hexencoded = list(map(lambda n: "%04x" % n, values))
        initvalues[i] = "".join(reversed(hexencoded))
    args = {
        "readwire": f'mem_read_data{memorymodule}',
        "read_addr": 'mem_read_address[7:0]',
        "read_enable": generate_proposition(memorymodule, 'mem_read_address', "mem_read_enable"),
        "write_enable": generate_proposition(memorymodule, "mem_write_address", "mem_write_enable"),
        "write_addr": 'mem_write_address[7:0]',
        "write_data": 'mem_write_data',
        "init": initvalues,
        "memorymodule": memorymodule
    }
    print(template.format(**args), file=ramfile)


with open("build/rcpu_ram.v", "w") as ramfile:
    for memorymodule in range(16):
        generate_module(code, memorymodule, ramfile)

    readwires = ' & '.join(f"mem_read_data{i}" for i in range(16))
    print(f"wire mem_read_data = {readwires};", file=ramfile)
