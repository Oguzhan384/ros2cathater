module adc_uart_daq (
    input  wire       CLOCK_50,

    output wire       ADC_CS_N,
    output wire       ADC_SCLK,
    output wire       ADC_DIN,
    input  wire       ADC_DOUT,

    output wire [0:0] GPIO_0
);

    // ============================================================
    // ADC
    // ============================================================

    wire [11:0] adc_raw;
    wire [12:0] voltage_mv;
    wire [24:0] mult_result;

    assign mult_result = adc_raw * 25'd5000;
    assign voltage_mv  = mult_result / 25'd4095;


    // ============================================================
    // BCD / UART
    // ============================================================

    wire [15:0] bcd_digits;

    wire       tx_busy;
    reg        tx_start = 1'b0;
    reg [7:0]  tx_data;
    reg [4:0]  state = 5'd0;


    // ============================================================
    // ADC SPI
    // ============================================================

    adc_spi_controller adc_inst (
        .clk_50  (CLOCK_50),
        .dout    (ADC_DOUT),
        .cs_n    (ADC_CS_N),
        .sclk    (ADC_SCLK),
        .din     (ADC_DIN),
        .adc_val (adc_raw)
    );


    // ============================================================
    // VPP MEASUREMENT
    //
    // ADC processing rate:
    // 50 MHz / 500 = 100 kHz
    //
    // Input signal:
    // 500 Hz
    //
    // Period:
    // 2 ms
    //
    // Window:
    // 4 ms = 2 full periods
    //
    // 400 samples per Vpp measurement
    // ============================================================

    reg [19:0] timer = 20'd0;

    // Light ADC IIR
    reg [12:0] smooth_inst_v = 13'd0;

    reg [12:0] v_max = 13'd0;
    reg [12:0] v_min = 13'd5000;

    reg [8:0] window_counter = 9'd0;

    reg [15:0] raw_vpp = 16'd0;
    reg [15:0] smoothed_vpp = 16'd0;

    reg vpp_ready = 1'b0;


    // ============================================================
    // LIGHT ADC IIR
    //
    // y[n] = 0.75*y[n-1] + 0.25*x[n]
    //
    // This only suppresses instantaneous ADC spikes.
    // ============================================================

    wire [15:0] smooth_next;

    assign smooth_next =
        ((smooth_inst_v * 3) + voltage_mv) >> 2;


    // ============================================================
    // ADAPTIVE VPP FILTER
    //
    // Large change:
    //     87.5% new + 12.5% old
    //
    // Small change:
    //     75% new + 25% old
    //
    // Threshold:
    //     100 mV
    // ============================================================

    reg [15:0] candidate_vpp;
    reg [15:0] difference_vpp;

    always @(*) begin

        // --------------------------------------------------------
        // Default candidate Vpp
        // --------------------------------------------------------

        candidate_vpp = v_max - v_min;


        // --------------------------------------------------------
        // Absolute difference between current Vpp and new Vpp
        // --------------------------------------------------------

        if (candidate_vpp >= smoothed_vpp)
            difference_vpp = candidate_vpp - smoothed_vpp;
        else
            difference_vpp = smoothed_vpp - candidate_vpp;

    end


    // ============================================================
    // MAIN PROCESS
    // ============================================================

    always @(posedge CLOCK_50) begin

        vpp_ready <= 1'b0;


        // ========================================================
        // 100 kHz processing tick
        // ========================================================

        if (timer == 20'd499) begin

            timer <= 20'd0;


            // ====================================================
            // LIGHT IIR
            // ====================================================

            smooth_inst_v <= smooth_next[12:0];


            // ====================================================
            // UPDATE MAXIMUM
            // ====================================================

            if (smooth_next > v_max)
                v_max <= smooth_next[12:0];


            // ====================================================
            // UPDATE MINIMUM
            // ====================================================

            if (smooth_next < v_min)
                v_min <= smooth_next[12:0];


            // ====================================================
            // END OF 4 ms WINDOW
            // ====================================================

            if (window_counter == 9'd399) begin

                // ------------------------------------------------
                // Calculate current window Vpp.
                // Include current sample if it creates a new
                // maximum or minimum.
                // ------------------------------------------------

                if (smooth_next > v_max) begin

                    raw_vpp <= smooth_next - v_min;

                end

                else if (smooth_next < v_min) begin

                    raw_vpp <= v_max - smooth_next;

                end

                else begin

                    raw_vpp <= v_max - v_min;

                end


                // ------------------------------------------------
                // Adaptive Vpp filtering
                //
                // IMPORTANT:
                // We calculate the current window's Vpp
                // explicitly instead of relying on raw_vpp,
                // because raw_vpp is a nonblocking register.
                // ------------------------------------------------

                if (smooth_next > v_max) begin

                    candidate_vpp = smooth_next - v_min;

                end

                else if (smooth_next < v_min) begin

                    candidate_vpp = v_max - smooth_next;

                end

                else begin

                    candidate_vpp = v_max - v_min;

                end


                // ------------------------------------------------
                // Calculate absolute difference
                // ------------------------------------------------

                if (candidate_vpp >= smoothed_vpp)
                    difference_vpp =
                        candidate_vpp - smoothed_vpp;
                else
                    difference_vpp =
                        smoothed_vpp - candidate_vpp;


                // ------------------------------------------------
                // LARGE CHANGE
                //
                // 87.5% new + 12.5% old
                //
                // Approx:
                //
                // old * 1/8 + new * 7/8
                // ------------------------------------------------

                if (difference_vpp > 16'd100) begin

                    smoothed_vpp <=
                        (smoothed_vpp +
                        (candidate_vpp * 7)) >> 3;

                end


                // ------------------------------------------------
                // SMALL CHANGE
                //
                // 75% new + 25% old
                //
                // old * 1/4 + new * 3/4
                // ------------------------------------------------

                else begin

                    smoothed_vpp <=
                        (smoothed_vpp +
                        (candidate_vpp * 3)) >> 2;

                end


                // ------------------------------------------------
                // Start next Vpp window
                //
                // Include current sample as first sample of
                // the new window.
                // ------------------------------------------------

                if (smooth_next > v_max)
                    v_max <= smooth_next[12:0];
                else
                    v_max <= 13'd0;


                if (smooth_next < v_min)
                    v_min <= smooth_next[12:0];
                else
                    v_min <= 13'd5000;


                window_counter <= 9'd0;

                vpp_ready <= 1'b1;

            end

            else begin

                window_counter <=
                    window_counter + 9'd1;

            end

        end

        else begin

            timer <= timer + 20'd1;

        end


        // ========================================================
        // UART TRANSMISSION
        // ========================================================

        if (state == 5'd0) begin

            if (vpp_ready)
                state <= 5'd1;

        end

        else begin

            tx_start <= 1'b0;


            case (state)

                // =================================================
                // THOUSANDS
                // =================================================

                5'd1: begin

                    if (!tx_busy) begin

                        tx_data <=
                            bcd_digits[15:12] + 8'd48;

                        tx_start <= 1'b1;

                        state <= 5'd2;

                    end

                end


                5'd2: begin

                    if (tx_busy)
                        state <= 5'd3;

                end


                // =================================================
                // HUNDREDS
                // =================================================

                5'd3: begin

                    if (!tx_busy) begin

                        tx_data <=
                            bcd_digits[11:8] + 8'd48;

                        tx_start <= 1'b1;

                        state <= 5'd4;

                    end

                end


                5'd4: begin

                    if (tx_busy)
                        state <= 5'd5;

                end


                // =================================================
                // TENS
                // =================================================

                5'd5: begin

                    if (!tx_busy) begin

                        tx_data <=
                            bcd_digits[7:4] + 8'd48;

                        tx_start <= 1'b1;

                        state <= 5'd6;

                    end

                end


                5'd6: begin

                    if (tx_busy)
                        state <= 5'd7;

                end


                // =================================================
                // ONES
                // =================================================

                5'd7: begin

                    if (!tx_busy) begin

                        tx_data <=
                            bcd_digits[3:0] + 8'd48;

                        tx_start <= 1'b1;

                        state <= 5'd8;

                    end

                end


                5'd8: begin

                    if (tx_busy)
                        state <= 5'd9;

                end


                // =================================================
                // CR
                // =================================================

                5'd9: begin

                    if (!tx_busy) begin

                        tx_data <= 8'h0D;

                        tx_start <= 1'b1;

                        state <= 5'd10;

                    end

                end


                5'd10: begin

                    if (tx_busy)
                        state <= 5'd11;

                end


                // =================================================
                // LF
                // =================================================

                5'd11: begin

                    if (!tx_busy) begin

                        tx_data <= 8'h0A;

                        tx_start <= 1'b1;

                        state <= 5'd12;

                    end

                end


                5'd12: begin

                    if (tx_busy)
                        state <= 5'd0;

                end


                default: begin

                    state <= 5'd0;

                end

            endcase

        end

    end


    // ============================================================
    // BINARY -> BCD
    // ============================================================

    bin2bcd bcd_inst (
        .bin (smoothed_vpp[12:0]),
        .bcd (bcd_digits)
    );


    // ============================================================
    // UART
    // ============================================================

    uart_tx #(
        .CLKS_PER_BIT(54)
    ) uart_inst (
        .clk      (CLOCK_50),
        .tx_start (tx_start),
        .tx_data  (tx_data),
        .tx_busy  (tx_busy),
        .tx_pin   (GPIO_0[0])
    );

endmodule


// =================================================================
// UART TRANSMITTER
// =================================================================

module uart_tx #(
    parameter CLKS_PER_BIT = 434
)(
    input  wire       clk,
    input  wire       tx_start,
    input  wire [7:0] tx_data,

    output reg        tx_busy,
    output reg        tx_pin
);

    reg [15:0] clk_count;
    reg [3:0]  bit_idx;
    reg [9:0]  shift_reg;
    reg [1:0]  state;


    initial begin

        tx_pin    = 1'b1;
        tx_busy   = 1'b0;
        state     = 2'd0;
        clk_count = 16'd0;
        bit_idx   = 4'd0;
        shift_reg = 10'd0;

    end


    always @(posedge clk) begin

        case (state)

            2'd0: begin

                if (tx_start) begin

                    shift_reg <= {
                        1'b1,
                        tx_data,
                        1'b0
                    };

                    clk_count <= 16'd0;
                    bit_idx   <= 4'd0;

                    tx_busy <= 1'b1;

                    state <= 2'd1;

                end

            end


            2'd1: begin

                tx_pin <= shift_reg[0];


                if (clk_count < CLKS_PER_BIT - 1) begin

                    clk_count <= clk_count + 16'd1;

                end

                else begin

                    clk_count <= 16'd0;


                    shift_reg <= {
                        1'b1,
                        shift_reg[9:1]
                    };


                    if (bit_idx < 4'd9) begin

                        bit_idx <= bit_idx + 4'd1;

                    end

                    else begin

                        tx_busy <= 1'b0;

                        state <= 2'd0;

                    end

                end

            end


            default: begin

                state <= 2'd0;

            end

        endcase

    end

endmodule


// =================================================================
// ADC SPI CONTROLLER
// =================================================================

module adc_spi_controller (

    input  wire        clk_50,
    input  wire        dout,

    output reg         cs_n = 1'b1,
    output wire        sclk,

    output reg         din = 1'b0,
    output reg [11:0]  adc_val = 12'd0
);

    reg [4:0] clk_div = 5'd0;


    // 50 MHz / 32 = 1.5625 MHz SCLK

    always @(posedge clk_50)
        clk_div <= clk_div + 5'd1;


    assign sclk = clk_div[4];


    reg [5:0] state = 6'd31;

    reg [15:0] shift_reg = 16'd0;


    // ============================================================
    // AD7928 CONTROL WORD
    //
    // RANGE = 0
    // ADC INPUT RANGE = 0 - 5 V
    // ============================================================

    wire [15:0] control_word =
        16'b1000001100010000;


    // ============================================================
    // CHIP SELECT
    // ============================================================

    always @(negedge sclk) begin

        state <= state + 6'd1;


        if (state == 6'd31) begin

            cs_n <= 1'b0;

            state <= 6'd0;

        end

        else if (state == 6'd16) begin

            cs_n <= 1'b1;

        end

    end


    // ============================================================
    // DATA TRANSFER
    // ============================================================

    always @(posedge sclk) begin

        if (state < 6'd16) begin

            din <= control_word[15 - state];

            shift_reg <= {
                shift_reg[14:0],
                dout
            };

        end

        else if (state == 6'd16) begin

            adc_val <= shift_reg[11:0];

        end

    end

endmodule


// =================================================================
// BINARY -> BCD
// =================================================================

module bin2bcd (

    input wire [12:0] bin,
    output reg [15:0] bcd

);

    integer i;


    always @(bin) begin

        bcd = 16'd0;


        for (i = 0; i < 13; i = i + 1) begin

            if (bcd[3:0] >= 5)
                bcd[3:0] = bcd[3:0] + 3;

            if (bcd[7:4] >= 5)
                bcd[7:4] = bcd[7:4] + 3;

            if (bcd[11:8] >= 5)
                bcd[11:8] = bcd[11:8] + 3;

            if (bcd[15:12] >= 5)
                bcd[15:12] = bcd[15:12] + 3;


            bcd = {
                bcd[14:0],
                bin[12-i]
            };

        end

    end

endmodule