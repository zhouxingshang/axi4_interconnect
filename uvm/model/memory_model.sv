// Simple memory model with read/write interface
class memory_model;
    bit[31:0] mem[bit[31:0]];
    function void write(bit[31:0] addr, bit[31:0] data);
        mem[addr[31:2]] = data;
    endfunction
    function bit[31:0] read(bit[31:0] addr);
        if(mem.exists(addr[31:2])) return mem[addr[31:2]];
        return 32'hDEAD_BEEF;
    endfunction
endclass
