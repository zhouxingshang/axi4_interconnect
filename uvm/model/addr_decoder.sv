//=============================================================================
// Address Decoder + 4KB Split Predictor
// Models: routing decode, split detection, split parameters, ID extension
//=============================================================================
class addr_decoder;

    //---- Split prediction result ----
    typedef struct {
        bit        will_split;         // burst crosses 4KB boundary
        bit[31:0]  sub1_addr;          // first sub-transaction start address
        bit[7:0]   sub1_len;           // first sub-transaction AWLEN (beats-1)
        bit[31:0]  sub2_addr;          // second sub-transaction start (next 4K page)
        bit[7:0]   sub2_len;           // second sub-transaction AWLEN (beats-1)
    } split_info_t;

    //---- Slave decode from address ----
    static function int decode(bit[31:0] addr);
        for(int i=0; i<4; i++)
            if(addr[31:13] == axi_pkg::SLV_ADDR_BASE[i][31:13]) return i;
        return 0;
    endfunction

    //---- 4KB boundary helpers ----
    static function bit[31:0] next_4k_page(bit[31:0] addr);
        return {addr[31:12] + 1'b1, 12'h0};
    endfunction

    static function bit crosses_4k(bit[31:0] addr, bit[7:0] len, bit[2:0] size);
        bit[31:0] end_addr;
        end_addr = addr + ((len + 1) << size) - 1;
        return addr[12] ^ end_addr[12];
    endfunction

    //---- Full split prediction ----
    // Returns: split_info_t with sub1/sub2 addr and len
    // If !will_split, sub2 fields are zero
    static function split_info_t predict_split(bit[31:0] addr, bit[7:0] len, bit[2:0] size);
        bit[12:0] bytes_to_bnd;     // bytes from addr to next 4K boundary
        bit[12:0] bytes_per_beat;
        bit[7:0]  beats_page1;      // beats that fit in first 4K page
        bit[7:0]  total_beats;

        predict_split.will_split = crosses_4k(addr, len, size);

        if(predict_split.will_split) begin
            total_beats    = len + 1;
            bytes_per_beat = 13'(1) << size;
            // bytes from addr to next 4K boundary (0x1000 aligned)
            bytes_to_bnd = 13'h1000 - {1'b0, addr[11:0]};
            // beats in first page = ceil(bytes_to_bnd / bytes_per_beat)
            beats_page1 = (bytes_to_bnd + bytes_per_beat - 13'd1) >> size;

            predict_split.sub1_addr = addr;
            predict_split.sub1_len  = beats_page1 - 1;           // len = beats - 1
            predict_split.sub2_addr = next_4k_page(addr);
            predict_split.sub2_len  = total_beats - beats_page1 - 1;
        end else begin
            predict_split.sub1_addr = addr;
            predict_split.sub1_len  = len;
            predict_split.sub2_addr = 0;
            predict_split.sub2_len  = 0;
        end
    endfunction

    //---- ID extension model ----
    // cross_4k_if adds 2-bit prefix: 2'b00=no-split, 2'b01=sub1, 2'b10=sub2
    // M2S prepends master index (2 bits) → S-side ID = {mst_idx, c4k_prefix, orig_id}
    static function bit[7:0] predict_sid(bit[1:0] mst_idx, bit[1:0] c4k_prefix, bit[3:0] orig_id);
        return {mst_idx, c4k_prefix, orig_id};
    endfunction

    // Given S-side ID, extract master index
    static function bit[1:0] get_mst_from_sid(bit[7:0] sid);
        return sid[7:6];
    endfunction

    // Given S-side ID, extract original ID
    static function bit[3:0] get_orig_from_sid(bit[7:0] sid);
        return sid[3:0];
    endfunction

endclass
