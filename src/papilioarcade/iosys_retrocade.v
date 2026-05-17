/*
 * iosys_retrocade.v
 *
 * FPGA Companion SPI-based IO system for snestang on Papilio Retrocade.
 * Replaces iosys_bl616 (UART-based) with an SPI-based interface that
 * communicates with the ESP32 FPGA Companion MCU.
 *
 * SPI pin mapping (m0s bus):
 *   m0s[0] = MISO  (FPGA -> MCU, tri-state when deselected)
 *   m0s[1] = MOSI  (MCU  -> FPGA, input)
 *   m0s[2] = SS/CS (MCU  -> FPGA, input, active-low)
 *   m0s[3] = CLK   (MCU  -> FPGA, input)
 *   m0s[4] = IRQn  (FPGA -> MCU, active-low interrupt)
 *   m0s[5] = flash_cs (tied HIGH)
 *
 * SPI targets:
 *   0 = SYS  (system control via sysctrl.v)
 *   1 = HID  (joystick/keyboard via hid.v)
 *   2 = OSD  (on-screen display, custom u8g2 bitmap renderer)
 *   3 = SDC  (SD card via sd_card.v)
 *
 * OSD buffer: 1024 bytes = 128x64 1bpp u8g2 bitmap.
 *   Address = page*128 + col, page = y/8, col = x.
 *   Bit K of buffer byte = pixel at (col, page*8+K).
 *   OSD is centered at overlay_x=[64,192), overlay_y=[80,144).
 *   2-cycle pipeline latency on hclk to match textdisp timing.
 *
 * SNES button format [11:0]:
 *   bit11=X, bit10=A, bit9=RT, bit8=LT, bit7=RIGHT, bit6=LEFT,
 *   bit5=DOWN, bit4=UP, bit3=START, bit2=SELECT, bit1=Y, bit0=B
 *
 * FPGA Companion HID joystick byte (joy[7:0]):
 *   bit0=right, bit1=left, bit2=down, bit3=up, bit4=A(Cross),
 *   bit5=B(Circle), bit6=X(Square), bit7=Y(Triangle)
 * extra_button byte:
 *   bit0=Select, bit1=Start, bit2=L1/LB, bit3=R1/RB
 *   (L2/R2 are bit4/5 but not mapped to 12-bit SNES format here)
 */

`timescale 1ns / 1ps

module iosys_retrocade #(
    parameter [7:0] CORE_ID = 8'h06,   // 6 = SNES
    parameter        FREQ    = 21_484_000
) (
    input            clk,              // main clock (mclk)
    input            hclk,             // HDMI pixel clock
    input            resetn,

    // SPI bus to ESP32 FPGA Companion MCU
    inout [5:0]      m0s,

    // Overlay interface to snes2hdmi (same as textdisp: 2-cycle latency on hclk)
    output reg       overlay,
    input  [7:0]     overlay_x,
    input  [7:0]     overlay_y,
    output reg [14:0] overlay_color,

    // HID joystick output (12-bit SNES format) to be OR'd with physical buttons
    output reg [11:0] hid1,
    output reg [11:0] hid2,

    // Physical SD card interface
    output           sd_clk,
    inout            sd_cmd,
    inout [3:0]      sd_dat,

    // ROM loading interface
    output reg       rom_loading,
    output reg [7:0] rom_do,
    output reg       rom_do_valid
);

    // =========================================================================
    // SPI signal routing
    // =========================================================================
    wire spi_io_ss   = m0s[2];    // active low
    wire spi_io_clk  = m0s[3];
    wire spi_io_din  = m0s[1];    // MOSI (MCU -> FPGA)
    wire spi_io_dout;              // MISO (FPGA -> MCU)
    wire irqn;                     // active-low IRQ to MCU

    assign m0s[0] = spi_io_ss ? 1'bz : spi_io_dout;  // MISO tri-state when deselected
    assign m0s[4] = irqn;
    assign m0s[5] = 1'b1;                              // flash_cs tied high

    wire reset = ~resetn;

    // =========================================================================
    // mcu_spi: byte-level SPI protocol routing
    // =========================================================================
    wire       mcu_start;
    wire [7:0] mcu_dout;

    wire       mcu_sys_strobe;
    wire       mcu_hid_strobe;
    wire       mcu_osd_strobe;
    wire       mcu_sdc_strobe;

    wire [7:0] mcu_sys_din;
    wire [7:0] mcu_hid_din;
    wire [7:0] mcu_osd_din  = 8'h00;  // OSD has no read-back
    wire [7:0] mcu_sdc_din;

    mcu_spi mcu_spi_inst (
        .clk           (clk),
        .reset         (reset),
        .spi_io_ss     (spi_io_ss),
        .spi_io_clk    (spi_io_clk),
        .spi_io_din    (spi_io_din),
        .spi_io_dout   (spi_io_dout),
        .mcu_sys_strobe(mcu_sys_strobe),
        .mcu_hid_strobe(mcu_hid_strobe),
        .mcu_osd_strobe(mcu_osd_strobe),
        .mcu_sdc_strobe(mcu_sdc_strobe),
        .mcu_start     (mcu_start),
        .mcu_sys_din   (mcu_sys_din),
        .mcu_hid_din   (mcu_hid_din),
        .mcu_osd_din   (mcu_osd_din),
        .mcu_sdc_din   (mcu_sdc_din),
        .mcu_dout      (mcu_dout)
    );

    // =========================================================================
    // hid.v: receive joystick data from FPGA Companion
    // =========================================================================
    wire [7:0] joystick0, joystick1;
    wire [7:0] extra_button0, extra_button1;
    wire       hid_irq, hid_iack;

    hid hid_inst (
        .clk            (clk),
        .reset          (reset),
        .data_in_strobe (mcu_hid_strobe),
        .data_in_start  (mcu_start),
        .data_in        (mcu_dout),
        .data_out       (mcu_hid_din),
        .db9_port       (6'h00),          // no DB9 port on Retrocade
        .irq            (hid_irq),
        .iack           (hid_iack),
        .joystick0      (joystick0),
        .joystick1      (joystick1),
        .extra_button0  (extra_button0),
        .extra_button1  (extra_button1),
        // Unused HID outputs (tied off)
        .numpad         (),
        .btn_select     (),
        .btn_start      (),
        .btn_b_w        (),
        .btn_diff_l     (),
        .btn_diff_r     (),
        .btn_pause      (),
        .mouse_btns     (),
        .mouse_x        (),
        .mouse_y        (),
        .mouse_strobe   (),
        .joystick0ax    (),
        .joystick0ay    (),
        .joystick1ax    (),
        .joystick1ay    (),
        .joystick_strobe(),
        // A2600-specific sysctrl inputs (not used for SNES)
        .p_dif1         (1'b0),
        .p_dif2         (1'b0),
        .p_color        (1'b0)
    );

    // =========================================================================
    // sysctrl.v: system interrupt and control
    // =========================================================================
    wire [7:0] int_in;
    wire [7:0] int_ack;
    wire       sdc_irq, sdc_iack;

    assign int_in   = {6'b000000, sdc_irq, hid_irq};
    assign hid_iack = int_ack[0];
    assign sdc_iack = int_ack[1];
    assign irqn     = ~(|int_in);   // active-low: assert when any interrupt pending

    sysctrl #(.CORE_ID(CORE_ID)) sysctrl_inst (
        .clk              (clk),
        .reset            (reset),
        .data_in_strobe   (mcu_sys_strobe),
        .data_in_start    (mcu_start),
        .data_in          (mcu_dout),
        .data_out         (mcu_sys_din),
        .int_out_n        (),              // IRQ driven directly from int_in above
        .int_in           (int_in),
        .int_ack          (int_ack),
        .buttons          (2'b00),
        // Unused sysctrl outputs
        .leds             (),
        .color            (),
        .port_status      (32'h00000000),
        .port_out_available(8'h00),
        .port_out_strobe  (),
        .port_out_data    (8'h00),
        .port_in_available(8'h00),
        .port_in_strobe   (),
        .port_in_data     (),
        .system_reset     (),
        .system_scanlines (),
        .system_volume    (),
        .system_screen    (),
        .system_port_1    (),
        .system_port_2    (),
        .system_video_std (),
        .system_paddle    (),
        .system_diff_p1   (),
        .system_diff_p2   (),
        .system_decomb    (),
        .system_vblank    (),
        .system_vm        (),
        .system_sc        (),
        .system_joyswap   ()
    );

    // =========================================================================
    // sd_card.v: physical SD card interface and sector-level ROM loading
    // =========================================================================
    wire        sd_outen;
    wire [8:0]  sd_outaddr;
    wire [7:0]  sd_outbyte;
    wire        sd_rbusy, sd_rdone;
    wire [63:0] sd_image_size;
    wire [7:0]  sd_image_mounted;
    wire [2:0]  sd_rsrc;

    reg  [7:0]  sd_rstart  = 8'h00;
    reg  [31:0] sd_rsector = 32'h00;

    sd_card #(
        .CLK_DIV (3'd2),
        .SIMULATE(1'b0)
    ) sd_card_inst (
        .rstn          (resetn),
        .clk           (clk),
        .sdclk         (sd_clk),
        .sdcmd         (sd_cmd),
        .sddat         (sd_dat),
        .data_strobe   (mcu_sdc_strobe),
        .data_start    (mcu_start),
        .data_in       (mcu_dout),
        .data_out      (mcu_sdc_din),
        .irq           (sdc_irq),
        .iack          (sdc_iack),
        .image_size    (sd_image_size),
        .image_mounted (sd_image_mounted),
        .rstart        (sd_rstart),
        .wstart        (8'h00),
        .rsector       (sd_rsector),
        .rsrc          (sd_rsrc),
        .rbusy         (sd_rbusy),
        .rdone         (sd_rdone),
        .inbyte        (8'h00),
        .outen         (sd_outen),
        .outaddr       (sd_outaddr),
        .outbyte       (sd_outbyte)
    );

    // =========================================================================
    // ROM Loader FSM: load ROM from SD card to snestang's loader interface
    //
    // Flow:
    //   1. MCU sends SDC CMD 4 (INSERTED) -> sd_image_mounted[0] pulses
    //   2. MCU sends SDC CMD 6 (DIRECT ACCESS) -> direct_start[0] set
    //   3. FSM asserts rstart[0], FPGA reads sectors from physical SD card
    //   4. sd_outen bytes forwarded to rom_do/rom_do_valid
    //   5. After image_size bytes, rom_loading deasserted
    // =========================================================================
    reg [2:0]  ld_state;
    localparam LD_IDLE        = 3'd0;
    localparam LD_REQUEST     = 3'd1;  // assert rstart, wait for busy
    localparam LD_READING     = 3'd2;  // forward bytes, wait for rdone
    localparam LD_NEXT_SECTOR = 3'd3;  // increment sector
    localparam LD_DONE        = 3'd4;  // clear rom_loading

    reg [63:0] ld_img_size;
    reg [63:0] ld_bytes_sent;
    reg [31:0] ld_sector;
    reg        ld_mounted_prev;

    wire ld_mounted_edge = sd_image_mounted[0] & ~ld_mounted_prev;

    always @(posedge clk) begin
        if (~resetn) begin
            ld_state       <= LD_IDLE;
            ld_sector      <= 32'h0;
            ld_bytes_sent  <= 64'h0;
            ld_img_size    <= 64'h0;
            ld_mounted_prev<= 1'b0;
            rom_loading    <= 1'b0;
            rom_do         <= 8'h00;
            rom_do_valid   <= 1'b0;
            sd_rstart      <= 8'h00;
            sd_rsector     <= 32'h0;
        end else begin
            ld_mounted_prev <= sd_image_mounted[0];
            rom_do_valid    <= 1'b0;
            sd_rstart       <= 8'h00;

            case (ld_state)
            LD_IDLE: begin
                if (ld_mounted_edge) begin
                    ld_img_size   <= sd_image_size;
                    ld_sector     <= 32'h0;
                    ld_bytes_sent <= 64'h0;
                    rom_loading   <= 1'b1;
                    ld_state      <= LD_REQUEST;
                end
            end

            LD_REQUEST: begin
                // Assert rstart[0] to request current sector
                sd_rstart  <= 8'h01;
                sd_rsector <= ld_sector;
                ld_state   <= LD_READING;
            end

            LD_READING: begin
                // Keep rstart asserted until sd_card captures the request.
                // sd_card checks rstart when core_request==CORE_REQ_IDLE.
                // Once it transitions to CORE_REQ_READ, rstart can be deasserted.
                // We deassert only when rdone fires (safe: rdone clears core_request).
                if (~sd_rbusy & ~sd_rdone) begin
                    // Still waiting for sd_card to start - keep rstart high
                    sd_rstart  <= 8'h01;
                    sd_rsector <= ld_sector;
                end

                // Forward each output byte from SD card to ROM loader
                if (sd_outen) begin
                    if (ld_bytes_sent < ld_img_size) begin
                        rom_do       <= sd_outbyte;
                        rom_do_valid <= 1'b1;
                        ld_bytes_sent <= ld_bytes_sent + 64'h1;
                    end
                end

                // Sector done
                if (sd_rdone) begin
                    sd_rstart <= 8'h00;
                    if (ld_bytes_sent >= ld_img_size)
                        ld_state <= LD_DONE;
                    else
                        ld_state <= LD_NEXT_SECTOR;
                end
            end

            LD_NEXT_SECTOR: begin
                ld_sector <= ld_sector + 32'h1;
                ld_state  <= LD_REQUEST;
            end

            LD_DONE: begin
                rom_loading <= 1'b0;
                ld_state    <= LD_IDLE;
            end

            default: ld_state <= LD_IDLE;
            endcase
        end
    end

    // =========================================================================
    // Joystick mapping: FPGA Companion HID -> 12-bit SNES button format
    //
    // SNES [11:0]: bit11=X, bit10=A, bit9=RT, bit8=LT, bit7=R, bit6=L,
    //              bit5=DOWN, bit4=UP, bit3=START, bit2=SELECT, bit1=Y, bit0=B
    //
    // FPGA Companion joystick byte:
    //   bit0=right, bit1=left, bit2=down, bit3=up, bit4=A(Cross),
    //   bit5=B(Circle), bit6=X(Square), bit7=Y(Triangle)
    // extra_button byte:
    //   bit0=Select, bit1=Start, bit2=L1, bit3=R1
    // =========================================================================
    always @(posedge clk) begin
        hid1 <= {
            joystick0[6],       // bit11: X  (Square)
            joystick0[4],       // bit10: A  (Cross)
            extra_button0[3],   // bit9:  RT (R1)
            extra_button0[2],   // bit8:  LT (L1)
            joystick0[0],       // bit7:  RIGHT
            joystick0[1],       // bit6:  LEFT
            joystick0[2],       // bit5:  DOWN
            joystick0[3],       // bit4:  UP
            extra_button0[1],   // bit3:  START
            extra_button0[0],   // bit2:  SELECT
            joystick0[7],       // bit1:  Y  (Triangle)
            joystick0[5]        // bit0:  B  (Circle)
        };
        hid2 <= {
            joystick1[6],
            joystick1[4],
            extra_button1[3],
            extra_button1[2],
            joystick1[0],
            joystick1[1],
            joystick1[2],
            joystick1[3],
            extra_button1[1],
            extra_button1[0],
            joystick1[7],
            joystick1[5]
        };
    end

    // =========================================================================
    // OSD: u8g2 bitmap renderer
    //
    // Buffer: 1024 bytes, address = {page[2:0], col[6:0]} = {oy[5:3], ox[6:0]}
    // where ox = overlay_x - 64, oy = overlay_y - 80 (OSD window 128x64 pixels)
    //
    // SPI OSD protocol (mcu_osd_strobe, mcu_start, mcu_dout):
    //   First byte after mcu_start = command:
    //     1 = SPI_OSD_ENABLE:  second byte sets osd_enabled
    //     2 = SPI_OSD_WRITE:   second byte = tile_addr; subsequent bytes fill buffer
    //
    // 2-cycle pipeline on hclk:
    //   Stage 1 (hclk): register address and control from combinatorial x/y
    //   Stage 2 (hclk): register BRAM output and compute overlay_color
    // =========================================================================

    // OSD buffer: 1024 bytes of block RAM
    reg [7:0] osd_buf [0:1023];

    // OSD SPI write state machine (runs on clk)
    reg        osd_enabled = 1'b0;
    reg [3:0]  osd_cmd_state;        // 0 = waiting for command, else byte count
    reg [7:0]  osd_cmd;
    reg [9:0]  osd_write_addr;

    always @(posedge clk) begin
        if (~resetn) begin
            osd_enabled   <= 1'b0;
            osd_cmd_state <= 4'h0;
        end else begin
            if (mcu_osd_strobe) begin
                if (mcu_start) begin
                    // First byte = command byte
                    osd_cmd       <= mcu_dout;
                    osd_cmd_state <= 4'h1;
                end else begin
                    if (osd_cmd_state != 4'hf)
                        osd_cmd_state <= osd_cmd_state + 4'h1;

                    // SPI_OSD_ENABLE = 1
                    if (osd_cmd == 8'h01) begin
                        if (osd_cmd_state == 4'h1)
                            osd_enabled <= mcu_dout[0];
                    end

                    // SPI_OSD_WRITE = 2
                    if (osd_cmd == 8'h02) begin
                        if (osd_cmd_state == 4'h1) begin
                            // Second byte = tile_addr: data_cnt = tile_addr * 8
                            osd_write_addr <= {mcu_dout[6:0], 3'b000};
                        end else begin
                            // Subsequent bytes fill the buffer
                            osd_buf[osd_write_addr] <= mcu_dout;
                            if (osd_write_addr != 10'h3ff)
                                osd_write_addr <= osd_write_addr + 10'h1;
                        end
                    end
                end
            end
        end
    end

    // OSD render pipeline (runs on hclk, 2-cycle latency to match textdisp)
    //
    // OSD window: overlay_x in [64,192), overlay_y in [80,144)
    // -> ox = overlay_x - 64  (0..127, 7 bits)
    // -> oy = overlay_y - 80  (0..63, 6 bits)
    // Buffer address = {oy[5:3], ox[6:0]} (3 + 7 = 10 bits)
    // Bit index = oy[2:0]

    wire        in_osd_comb = (overlay_x >= 8'd64)  && (overlay_x < 8'd192) &&
                               (overlay_y >= 8'd80)  && (overlay_y < 8'd144);
    wire [6:0]  ox_comb     = overlay_x - 8'd64;   // 8-bit sub, result fits in 7 bits (0..127)
    wire [5:0]  oy_comb     = overlay_y - 8'd80;   // 8-bit sub, result fits in 6 bits (0..63)
    wire [9:0]  osd_addr    = {oy_comb[5:3], ox_comb[6:0]};
    wire [2:0]  osd_bit     = oy_comb[2:0];

    // Stage 1 pipeline registers
    reg [9:0]  osd_addr_r1;
    reg [2:0]  osd_bit_r1;
    reg        in_osd_r1;

    // Stage 2 pipeline registers (BRAM data captured here)
    reg [7:0]  osd_data_r2;
    reg [2:0]  osd_bit_r2;
    reg        in_osd_r2;

    always @(posedge hclk) begin
        // Stage 1: register address and control signals
        osd_addr_r1 <= osd_addr;
        osd_bit_r1  <= osd_bit;
        in_osd_r1   <= in_osd_comb;

        // Stage 2: read from OSD buffer using stage-1 registered address
        osd_data_r2 <= osd_buf[osd_addr_r1];
        osd_bit_r2  <= osd_bit_r1;
        in_osd_r2   <= in_osd_r1;

        // Output: compute overlay_color from stage-2 data
        if (in_osd_r2 && osd_data_r2[osd_bit_r2])
            overlay_color <= 15'h7FFF;   // white pixel
        else
            overlay_color <= 15'h0000;   // black pixel (or outside OSD)
    end

    // overlay enable: driven from OSD state machine
    always @(posedge clk) begin
        overlay <= osd_enabled;
    end

endmodule
