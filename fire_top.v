`timescale 1ns / 1ps

module pwm_Nstep_freq3 #(  /////////////////////////////////////////////////Nstep_freq2�� ������. ���߿� 2 ������       
        parameter sys_clk_freq = 100_000_000,
        parameter pwm_freq = 10_000,
        parameter duty_step = 100,
        parameter temp = sys_clk_freq / duty_step / pwm_freq,
        parameter temp_half = temp / 2  //�̸� ���Ǿ ������ ����ȴ�.
    )(      
        input clk, reset_p,
        input [31:0] duty,
        output pwm);
                       
        integer cnt_sysclk;       
        wire clk_freqXstep;
        always @(negedge clk or posedge reset_p)begin
                if(reset_p)cnt_sysclk = 0;
                else begin
                        if(cnt_sysclk >= temp-1) cnt_sysclk = 0;
                        else cnt_sysclk = cnt_sysclk + 1;
                 end
        end
        
        assign clk_freqXstep = (cnt_sysclk < temp_half) ? 1 : 0; 
        wire clk_freqXstep_nedge;
        
          edge_detector_n ed(
             .clk(clk), .reset_p(reset_p), .cp(clk_freqXstep),
             .n_edge(clk_freqXstep_nedge));

         integer cnt_duty;
        
        always @(negedge clk or posedge reset_p)begin
                if(reset_p)cnt_duty = 0;
                else if(clk_freqXstep_nedge) begin
                    if(cnt_duty >= (duty_step-1))cnt_duty = 0;   
                    else cnt_duty = cnt_duty + 1;
                 end
        end
        
        assign pwm = (cnt_duty < duty) ? 1 : 0;    //0 10�� 25% 10�� ~     <=�̸� 0���ΰ� ����� 
        //1�ʿ� 100hz�� 10�� �ݺ�  
         
endmodule





//----------���ܼ� �������----------------------------------------------------------


module ir_sensor_top(
    input clk, 
    input reset_p,
    input ir_input,         // FC-51 ������ ������ ��� �Է�          
    input count_enable,     // ī��Ʈ ���� ���� ��ȣ  
    output reg no_object_detected // ��ü ���� ���� ���
);
    // 5�� Ÿ�̸Ӹ� ���� ī���� (100 MHz Ŭ�� ����, 5�� = 500,000,000 ����Ŭ)
    reg [28:0] no_object_counter; // 29��Ʈ ī���� (2^29 > 500,000,000)
    localparam TIMEOUT = 29'd500_000_000; // 5�� (100MHz Ŭ�� ����)

    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            no_object_counter <= 29'd0;
            no_object_detected <= 1'b0;
        end
         else if (!count_enable) begin
            // ī��Ʈ�� ��Ȱ��ȭ ���¶�� ī���Ϳ� ���� ���� �ʱ�ȭ
            no_object_counter <= 29'd0;
            no_object_detected <= 1'b0;
         end
        else if(count_enable)begin
            if (ir_input == 1'b0) begin
                // ��ü�� �����Ǹ� ī���� �ʱ�ȭ
                no_object_counter <= 29'd0;
                no_object_detected <= 1'b0;
            end 
            else if (no_object_counter < TIMEOUT) begin
                // ��ü�� �������� ������ ī���� ����
                no_object_counter <= no_object_counter + 1;
            end 
            else begin
                // 5�� ���� ��ü�� �������� ������ ���� ��ȣ ���
                no_object_detected <= 1'b1;
            end
        end
    end
endmodule



//----------�������Ϳ� ���ܼ� ���� ��� ��ģ�� ------------------------------------------



module fire_detect_top (
    input clk,
    input reset_p,    
    input sw_v1,    // ����ġ �Է� (����ġ�� �ö󰡸� IR ���� Ȱ��ȭ)
    input sw_v2,
    inout dht11_data,
    input ir_input1,  // ���ܼ� ������ ��ȣ �Է�1
    input ir_input2, //���ܼ� ������ ��ȣ �Է�2
    input vauxp6, vauxn6, vauxp15, vauxn15, vauxp14, vauxn14,
    output scl,sda,
    output sv_pwm1,
    output sv_pwm2,    // �������� ���� PWM ��ȣ ���
    output reg neo_pin,
    output  neo_pin1, neo_pin2, neo_pin3, neo_pin4, neo_pin5, neo_pin6, neo_pin7,
    output reg buzz
);

    wire no_object_detected1, no_object_detected2;  // ��ü�� 5�� ���� �������� ���� ���
    reg [6:0] duty1, duty2;           // �������� ��� ���� ��Ƽ����Ŭ
    reg [6:0] duty_min, duty_max;
    reg down_up1, down_up2;              // ���� ���� �÷���
    reg ir_active1, ir_active2;            // ���ܼ� ���� Ȱ��ȭ ��ȣ

    reg [31:0] clk_div;
    wire clk_div_21_nedge;
    
    reg sw_pedge1, sw_pedge2;
    
     wire [15:0] bcd_f1, bcd_f2, bcd_f3;

    fire_flame_sensor(
    .clk(clk), .reset_p(reset_p),
    .vauxp6(vauxp6), .vauxn6(vauxn6), .vauxp15(vauxp15), .vauxn15(vauxn15), .vauxp14(vauxp14), .vauxn14(vauxn14),
    .dht11_data(dht11_data),
    .scl(scl),.sda(sda),
    .bcd_f1(bcd_f1),.bcd_f2(bcd_f2),.bcd_f3(bcd_f3)
    );
    
    reg fire_flag;

    // �ʱ�ȭ �� clk_div ����
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
             clk_div <= 0;              
        end 
        else begin
            clk_div <= clk_div + 1;
        end
    end

   // 20ms �ֱ⸦ �����ϴ� edge detector
    edge_detector_n ed(
        .clk(clk),
        .reset_p(reset_p),
        .cp(clk_div[21]),
        .n_edge(clk_div_21_nedge)
    );
    
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin         
            duty1 <= 12; 
            duty2 <= 12;     
            duty_min <= 12;       // �ּ� ��Ƽ����Ŭ (0��)
            duty_max <= 31;       // �ִ� ��Ƽ����Ŭ (180��)   
            down_up1 <= 0;         // �ʱ� ���� ���� (0: ����, 1: ����)
            down_up2 <= 0;
            ir_active1 <= 0;       // IR ���� ��Ȱ��ȭ
            ir_active2 <= 0;
            sw_pedge1 <= 0;       // LED ��ȣ flag
            sw_pedge2 <= 0;
            fire_flag <= 0;
            neo_pin <= 0;
            buzz <= 0;
        end 
        else if (clk_div_21_nedge) begin
            if (bcd_f1[7:0] < 20 || bcd_f2[7:0] < 20 || bcd_f3[7:0] < 20) begin  // ����ġ�� �ö󰡸� IR ���� Ȱ��ȭ
                sw_pedge1 <= 1;
                ir_active1 <= 1;   // IR ���� Ȱ��ȭ
                fire_flag <= 1;
                buzz <= 1;
            end
            else begin
                 if (no_object_detected1) begin // 5�� ���� ������ �ȵǸ�
                    if (!down_up1) begin  // ���������� ������ ���� ������ ��
                         if (duty1 < duty_max) begin  // �ִ�(180��)���� �������͸� ������Ŵ
                            duty1 <= duty1 + 1;  // ���� ����
                        end
                        else if (duty1 == duty_max) begin
                            down_up1 <= 1; // �ִ�ġ�� �����ϸ� �÷��� ����
                        end
                    end                
                end
                else if (!no_object_detected1) begin  // ���� ������ �ٽ� ��ü�� �����Ǹ�
                    if (duty1 < duty_max) begin
                        duty1 <= duty_min;
                    end
                    else if (duty1 == duty_max) begin
                        down_up1 <= 1;
                    end                    
                end
            end

            if (bcd_f1[7:0] < 20 || bcd_f2[7:0] < 20 || bcd_f3[7:0] < 20) begin  // ����ġ�� �ö󰡸� IR ���� Ȱ��ȭ
                sw_pedge2 <= 1;
                ir_active2 <= 1;   // IR ���� Ȱ��ȭ
            end
            else begin
                if (no_object_detected2) begin // 5�� ���� ������ �ȵǸ�
                    if (!down_up2) begin  // ���������� ������ ���� ������ ��
                         if (duty2 < duty_max) begin  // �ִ�(180��)���� �������͸� ������Ŵ
                            duty2 <= duty2 + 1;  // ���� ����
                        end
                        else if (duty2 == duty_max) begin
                            down_up2 <= 1; // �ִ�ġ�� �����ϸ� �÷��� ����
                        end
                    end                
                end
                else if (!no_object_detected2) begin  // ���� ������ �ٽ� ��ü�� �����Ǹ�
                    if (duty2 < duty_max) begin
                        duty2 <= duty_min;
                    end
                    else if (duty2 == duty_max) begin
                        down_up2 <= 1;
                    end                    
                end
            end
        end
    end

    // IR ���� ��� �ν��Ͻ�
    ir_sensor_top ir_sensor_inst1 (
        .clk(clk),
        .reset_p(reset_p),
        .ir_input(ir_input1),              
        .count_enable(ir_active1),   // ����ġ�� ���� ���¿����� ī��Ʈ Ȱ��ȭ
        .no_object_detected(no_object_detected1)
    );
    
     ir_sensor_top ir_sensor_inst2 (
        .clk(clk),
        .reset_p(reset_p),
        .ir_input(ir_input2),              
        .count_enable(ir_active2),   // ����ġ�� ���� ���¿����� ī��Ʈ Ȱ��ȭ
        .no_object_detected(no_object_detected2)
    );
    
    // LCD
     i2c_txt_warning_top(
    .clk(clk), .reset_p(reset_p),
    .fire_flag(fire_flag),
    .dht11_data(dht11_data),
    .scl(scl), .sda(sda),
    .com(com),
    .seg_7(seg_7)
    );


    // �������� ���� PWM ��� �ν��Ͻ�
    pwm_Nstep_freq3 #(
        .duty_step(400),  // 400�ܰ�� ����
        .pwm_freq(50)     // PWM ���ļ� 50Hz
    ) pwm_motor1 (
        .clk(clk),
        .reset_p(reset_p),
        .duty(duty1),
        .pwm(sv_pwm1)
    );
    
    pwm_Nstep_freq3 #(
        .duty_step(400),  // 400�ܰ�� ����
        .pwm_freq(50)     // PWM ���ļ� 50Hz
    ) pwm_motor2 (
        .clk(clk),
        .reset_p(reset_p),
        .duty(duty2),
        .pwm(sv_pwm2)
    );
    
    wire [4:0] led_state;
    neopixel_cntr1 fire_led1(
        .clk(clk), .reset_p(reset_p),
        .start_signal(sw_pedge1 || sw_pedge2),
        .state(led_state)
    );

    assign  neo_pin1 = led_state[0];
    assign  neo_pin2 = led_state[1];
    assign  neo_pin3 = led_state[2];
    assign  neo_pin4 = led_state[3];
    assign  neo_pin5 = led_state[4];
    assign  neo_pin6 = led_state[4];
    assign  neo_pin7 = led_state[4];
        
endmodule


//---------------------------------------------------------------------

module neopixel_cntr1(
    input clk, reset_p,
    input start_signal,
    output reg [4:0] state
);
    
    wire clk_usec, clk_msec, clk_60msec, clk_600msec;

     clock_div_100 usec_clk (
        .clk(clk), .reset_p(reset_p), // 1us �ֱ�� ����
        .clk_div_100(clk_usec)
    );

    clock_div_1000 msec_clk (
        .clk(clk), .reset_p(reset_p),
        .clk_source(clk_usec), // 1ms �ֱ�� ����
        .clk_div_1000(clk_msec)
    );

    clock_div_60 sixty_msec_clk (
        .clk(clk), .reset_p(reset_p),
        .clk_source(clk_msec), // 60ms �ֱ�� ����
        .clk_div_60_nedge(clk_60msec)
    );

    clock_div_10 six_hundreds_msec_clk (
        .clk(clk), .reset_p(reset_p),
        .clk_source(clk_60msec), // 600ms (0.6s) �ֱ�� ����
        .clk_div_10_nedge(clk_600msec)
    );

    
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) state = 5'b11111;
        else if (start_signal && clk_600msec) begin
            case (state)
                5'b11111, 
                5'b11110, 
                5'b11100, 
                5'b11000, 
                5'b10000 :
                state = {state[3:0], 1'b0};
                
                5'b00000, 
                5'b00001, 
                5'b00011, 
                5'b00111, 
                5'b01111 :
                state = {state[3:0], 1'b1};
            endcase
        end
        else if (!start_signal) begin
            state = 5'b11111;
        end
    end

endmodule



//module neopixel_cntr2(
//    input clk, reset_p,
//    input start_signal,
//    output reg neo_pin
//);

////    reg start_stop; // LED��Ʈ�� ����: 1 => start, 0 => stop
//    always@ (posedge clk or posedge reset_p) begin
//        if (reset_p) neo_pin = 1;
        
//        else if (start_signal) begin // 0�� ��ư ������ 1 => start, �� �� �� ������ 0 => stop
//            neo_pin = 0;
//        end
//        else if (!start_signal) begin
//            neo_pin = 1;
//        end
//    end

//endmodule


//---------------------------------------------------------------------------------------
module i2c_txt_warning_top(
    input clk, reset_p,
    input fire_flag,
    inout dht11_data,
    output scl, sda,
    output [3:0] com,
    output [7:0] seg_7,
    output [15:0] led_debug);
    
    parameter IDLE                            = 10'b00000_00001; 
    parameter INIT                            = 10'b00000_00010; 
    parameter SEND_TEMP                       = 10'b00000_00100;
    parameter SEND_COMMAND                    = 10'b00000_01000;
    parameter SEND_HUMIDITY                   = 10'b00000_10000;
    parameter SEND_second_line                = 10'b00001_00000;
    parameter SEND_WARNING                    = 10'b00010_00000;
    parameter SEND_DELAY                      = 10'b00100_00000;
    parameter SEND_retrun_home                = 10'b01000_00000;
    parameter SEND_CLEAR                      = 10'b10000_00000;

    wire clk_usec;
    clock_div_100 usec_clk (
        .clk(clk), .reset_p(reset_p), // �ùķ��̼� �� �� 10ns�� �ָ� ==> 100���� ���� 1000ns = 1us �ֱ�� ����
        .clk_div_100_nedge(clk_usec));
    
    reg [21:0] count_usec;  // 3sec = 3,000,000usec, 3,000,000�� 22��Ʈ
    reg count_usec_e;
    always @(negedge clk or posedge reset_p) begin
        if (reset_p) count_usec = 0;
        else if (clk_usec && count_usec_e) count_usec = count_usec + 1;
        else if (!count_usec_e) count_usec = 0; 
    end
    
    reg [7:0] send_buffer;
    reg rs, send;   // rs 0 = command, 1 = data
    wire busy;
    i2c_lcd_send_byte txtlcd(
        .clk(clk), .reset_p(reset_p),
        .addr(7'h27),
        .send_buffer(send_buffer),
        .rs(rs), .send(send),
        .scl(scl), .sda(sda),
        .busy(busy),
        .led(led_debug));

    wire [7:0] humidity, temperature;

    dht11_cntr rh_dh(
    .clk(clk), .reset_p(reset_p),
    .dht11_data(dht11_data), 
    .humidity(humidity), 
    .temperature(temperature),
    .led_debug(led_debug)
    );

    wire [15:0] fnd_value;
    wire [15:0] humidity_bcd, temperature_bcd;

     bin_to_dec bcd_humidity(
            .bin({4'b0, humidity}), 
            .bcd(humidity_bcd)
        );
        bin_to_dec bcd_temperature(
            .bin({4'b0, temperature}), 
            .bcd(temperature_bcd)
        );
    
    reg [10:0] state, next_state;
    always @(negedge clk or posedge reset_p) begin
        if (reset_p) state = IDLE;
        else state = next_state;
    end

    assign led_debug[15:6] = state;
    
    // ���� ���� �߰�
    reg [9:0] previous_state;
    reg [8*4-1:0] temperature_str;
    reg [8*3-1:0] humidity_str;
    reg [8*4-1:0] temp;
    reg [8*2-1:0] rh;
    reg init_flag;
    reg [3:0] cnt_data;
    reg [8*5-1:0] fire;
    reg [3:0] cnt_string;
    integer i;

    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            next_state = IDLE;
            init_flag = 0;
            cnt_data = 0;
            rs = 0;
            fire = "FIRE!";
            temp = "TEMP";
            rh = "RH";
            cnt_string = 0;
            previous_state = IDLE;
        end
        else begin
            case(state)
                IDLE:begin
                    if (init_flag)begin // �ʱ� init_flag = 0
                        if (fire_flag) next_state = SEND_CLEAR;    // down
                        else next_state = SEND_TEMP;
                    end
                    else begin
                        if (count_usec <= 22'd80_000)begin // 80ms�� ���
                            count_usec_e = 1;
                        end
                        else begin
                            next_state = INIT;
                            count_usec_e = 0;
                        end
                    end
                end
                INIT:begin
                    if (busy)begin // �ʱ� busy�� 0
                        send = 0;
                        if (cnt_data >= 6)begin
                            next_state = IDLE;
                            init_flag = 1;
                            cnt_data = 0;
                        end
                    end
                    else if (!send)begin
                        case(cnt_data)
                            0: send_buffer = 8'h33;
                            1: send_buffer = 8'h32;
                            2: send_buffer = 8'h28;
                            3: send_buffer = 8'h0f; // display, cursor, blink
                            4: send_buffer = 8'h01; // clear
                            5: send_buffer = 8'h06; // ���������� ����
                        endcase
                        rs = 0; // send command;
                        send = 1;
                        cnt_data = cnt_data + 1;
                    end
                end
              
                SEND_TEMP: begin
                    if (busy) begin
                        send = 0;
                        if(cnt_string >= 9) begin 
                            next_state = SEND_DELAY;
                            previous_state = SEND_TEMP;  // ���� ���� ����
                            cnt_string = 0;
                        end
                    end
                    else if (!send) begin
                        case(cnt_string)
                            0: send_buffer = temp[31:24];
                            1: send_buffer = temp[23:16];
                            2: send_buffer = temp[15:8];
                            3: send_buffer = temp[7:0];
                            4: send_buffer = 8'h20; // ����
                            5: send_buffer = "0" + temperature_bcd[7:4];
                            6: send_buffer = "0" + temperature_bcd[3:0];
                            7: send_buffer = 8'h27;
                            8: send_buffer = 8'h43;
                            endcase
                        rs = 1;  // ������ ����
                        send = 1;
                        cnt_string = cnt_string + 1;
                    end
                end
                SEND_DELAY: begin
                    if(count_usec <= 22'd300_000)begin
                        count_usec_e = 1;
                    end
                    else begin
                            count_usec_e = 0;
                            case (previous_state)
                                SEND_TEMP: next_state = SEND_second_line;
                                SEND_HUMIDITY: next_state = SEND_retrun_home;
                                SEND_retrun_home: next_state = IDLE;
                                SEND_CLEAR: next_state = SEND_WARNING;
                                SEND_second_line: next_state = SEND_HUMIDITY;
                                SEND_WARNING: next_state = SEND_retrun_home;
                                default: next_state = IDLE;
                            endcase
                    end
                end
                SEND_retrun_home: begin
                    if(busy) begin
                        send = 0;
                        next_state = SEND_DELAY;
                        previous_state = SEND_retrun_home;
                    end
                    else begin
                        send_buffer = 8'h02; // clear
                        rs = 0; // ��ɾ� �����̹Ƿ� rs�� 0���� ����
                        send = 1;
                    end
                end
                SEND_second_line: begin
                    if(busy) begin
                        send = 0;
                        next_state = SEND_DELAY;
                        previous_state = SEND_second_line;
                    end
                    else begin
                        send_buffer = 8'hC0; // �����ٷ� �̵�
                        rs = 0; // ��ɾ� �����̹Ƿ� rs�� 0���� ����
                        send = 1;
                    end
                end
                SEND_CLEAR: begin
                    if(busy) begin
                        send = 0;
                        next_state = SEND_DELAY;
                        previous_state = SEND_CLEAR;
                    end
                    else begin
                        send_buffer = 8'h01; // �����ٷ� �̵�
                        rs = 0; // ��ɾ� �����̹Ƿ� rs�� 0���� ����
                        send = 1;
                    end
                end
                SEND_HUMIDITY:begin
                    if (busy) begin
                        send = 0;
                        if(cnt_string >= 9) begin 
                            next_state = SEND_DELAY;
                             previous_state = SEND_HUMIDITY;  // ���� ���� ����
                            cnt_string = 0;
                        end
                    end
                    else if (!send)begin
                        case(cnt_string)
                            0: send_buffer = rh[15:8];
                            1: send_buffer = rh[7:0];
                            2, 3, 4, 5: send_buffer = 8'h20; // ���� ���� ����
                            6: send_buffer = "0" + humidity_bcd[7:4];
                            7: send_buffer = "0" + humidity_bcd[3:0];
                            8: send_buffer = 8'h25; // "%" ��ȣ ����
                        endcase
                        rs = 1; // send data
                        send = 1;
                        cnt_string = cnt_string + 1;
                    end
                end
                SEND_WARNING:begin
                    if(busy)begin
                        send = 0;
                        if (cnt_string >= 5)begin
                            next_state = SEND_DELAY;
                            previous_state = SEND_WARNING;
                            cnt_string = 0;
                        end
                    end
                    else if (!send)begin
                        case(cnt_string)
                            0: send_buffer = fire[39:32];
                            1: send_buffer = fire[31:24];
                            2: send_buffer = fire[23:16];
                            3: send_buffer = fire[15:8];
                            4: send_buffer = fire[7:0];
                        endcase
                        rs = 1; // send data
                        send = 1;
                        cnt_string = cnt_string + 1;
                    end
                end
            endcase
        end
    end

    assign fnd_value = {humidity_bcd[7:0], temperature_bcd[7:0]};
    fnd_cntr fnd( .clk(clk), .reset_p(reset_p), .value(fnd_value), .com(com), .seg_7(seg_7));
endmodule

//---------------------------------------------------------------------------------------
module fire_flame_sensor(
   input clk, reset_p,
   input vauxp6, vauxn6, vauxp15, vauxn15, vauxp14, vauxn14,
   inout dht11_data,
   output scl,sda,
   output led_r,
   output [3:0] com,
   output [7:0] seg_7,
   output [15:0] bcd_f1,bcd_f2,bcd_f3
);

 ////////////////// flame ���� //////////////////
   wire [4:0] channel_out;
   wire [15:0] do_out;
   wire eoc_out;
   wire [15:0] adc_value;
     
   fire_detect adc_fire(
          .daddr_in({2'b0, channel_out}),            // Address bus for the dynamic reconfiguration port
          .dclk_in(clk),             // Clock input for the dynamic reconfiguration port
          .den_in(eoc_out),              // Enable Signal for the dynamic reconfiguration port
          .reset_in(reset_p),            // Reset signal for the System Monitor control logic
          .vauxp6(vauxp6),              // Auxiliary channel 6
          .vauxn6(vauxn6),
          .vauxp14(vauxp14),
          .vauxn14(vauxn14),
          .vauxp15(vauxp15),          
          .vauxn15(vauxn15),
          .channel_out(channel_out),    // � ä�η� �����ߴ��� �˷���
          .do_out(do_out),              // enable�� ��� ����� ������ ��������
          .eoc_out(eoc_out)             // ��ȯ �۾��� ������ ��
    );
    
   wire eoc_out_pedge;
   edge_detector_n ed_btn0(
       .clk(clk), .reset_p(reset_p),
       .cp(eoc_out),
       .p_edge(eoc_out_pedge)
   );
    
   reg [11:0] flame1, flame2, flame3;
    
   always @(negedge clk or posedge reset_p) begin
       if (reset_p) begin
           flame1 = 0;
           flame2 = 0;
           flame3 = 0;
       end
       else if (eoc_out_pedge) begin
           case (channel_out[3:0]) // channel_out�� �ֻ��� ��Ʈ(4��°)�� ��� �����̱� ������ ����
               6: flame1 = do_out[15:4];
               14: flame3 = do_out[15:4];
               15: flame2 = do_out[15:4];
           endcase
       end
   end
    
    wire [15:0] value;
    
   bin_to_dec bcd_adc_f1(
       .bin({6'b0, flame1[11:6]}), // ���� ���� ������ �߻� ==> �ؿ� ��Ʈ�� �����ϰ� ���� 12��Ʈ ���
       .bcd(bcd_f1)
   );
    
   bin_to_dec bcd_adc_f2(
       .bin({6'b0, flame2[11:6]}), // ���� ���� ������ �߻� ==> �ؿ� ��Ʈ�� �����ϰ� ���� 12��Ʈ ���
       .bcd(bcd_f2)
   );

   bin_to_dec bcd_adc_f3(
       .bin({6'b0, flame2[11:6]}), // ���� ���� ������ �߻� ==> �ؿ� ��Ʈ�� �����ϰ� ���� 12��Ʈ ���
       .bcd(bcd_f3)
   );
     
endmodule


