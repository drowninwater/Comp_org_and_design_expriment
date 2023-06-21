`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2023/06/21 14:56:40
// Design Name: 
// Module Name: top
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module top(
    input rstn,
    input [4:0] btn_i,
    input [15:0] sw_i,
    input clk,
    
    output [7:0] disp_an_o,
    output [7:0] disp_seg_o,
    
    output [15:0] led_o
    );
    
    wire rst;
    wire [4:0] BTN_out;
    wire [15:0] SW_out;
    wire [31:0] clkdiv;
    wire Clk_CPU;
    wire Clk_CPU_N;
    wire GPIOf0000000_we;
    wire [31:0] Peripheral_in;
    wire [1:0] counter_set;
    wire [15:0] led;
    wire [15:0] LED_out;
    wire [31:0] Addr_out;
    wire mem_w;
    wire [2:0] dm_ctrl;
    wire [31:0] Cpu_data4bus;
    wire [31:0] ram_data_in;
    wire [31:0] Data_read;
    wire [31:0] Data_write_to_dm;
    wire [3:0] wea_mem;
    wire clkn;
    wire [9:0] ram_addr;
    wire [31:0] douta;
    wire GPIOe0000000_we;
    wire [63:0] point_in;
    wire counter_we;
    wire counter0_OUT;
    wire counter1_OUT;
    wire counter2_OUT;
    wire [31:0] PC_out;
    wire [31:0] spo;
    wire CPU_MIO;
    wire [31:0] data1;
    wire [31:0] Data_out;
    wire [63:0] LES;
    wire [31:0] counter_out;
    wire [7:0] point_out;
    wire [7:0] LE_out;
    wire [31:0] Disp_num;
    
    assign rst = ~rstn;
    assign clkn = ~clk;
    assign Clk_CPU_N = ~Clk_CPU;
    assign point_in = {clkdiv[31:0], clkdiv[31:0]};
    assign data1 = {2'b0, PC_out[31:2]};
    assign LES = 64'hFFFFFFFFFFFFFFFF;
    
    Enter U10_Enter(.clk(clk),
                    .BTN(btn_i),
                    .SW(sw_i),
                    .BTN_out(BTN_out),
                    .SW_out(SW_out)
                    );
    
    clk_div U8_clk_div(.clk(clk),
                       .rst(rst),
                       .SW2(SW_out[2]),
                       .clkdiv(clkdiv),
                       .Clk_CPU(Clk_CPU)
                       );
                       
    
    SPIO U7_SPIO(.clk(Clk_CPU_N),
                 .rst(rst),
                 .EN(GPIOf0000000_we),
                 .P_Data(Peripheral_in),
                 .counter_set(counter_set),
                 .LED_out(LED_out),
                 .led(led) //GPIOf0???
                );
    
    dm_controller U3_dm_controller(.mem_w(mem_w),
                                   .Addr_in(Addr_out),
                                   .Data_write(ram_data_in),
                                   .dm_ctrl(dm_ctrl),
                                   .Data_read_from_dm(Cpu_data4bus),
                                   .Data_read(Data_read),
                                   .Data_write_to_dm(Data_write_to_dm),
                                   .wea_mem(wea_mem)
                                   );
    
    Counter_x U9_Counter_x(.clk(Clk_CPU_N),
                           .rst(rst),
                           .clk0(clkdiv[6]),
                           .clk1(clkdiv[9]),
                           .clk2(clkdiv[11]),
                           .counter_we(counter_we),
                           .counter_val(Peripheral_in),
                           .counter_ch(counter_set),
                           .counter0_OUT(counter0_OUT),
                           .counter1_OUT(counter1_OUT),
                           .counter2_OUT(counter2_OUT)
                           );
    
    ROM_0 U2_ROMD(.a(PC_out[11:2]),
                  .spo(spo)
                  );
    
    RAM_B U4_RAM_B(.clka(clkn),
                   .wea(wea_mem),
                   .addra(ram_addr),
                   .dina(Data_write_to_dm),
                   .douta(douta)
                   );
    
    SCPU U1_SCPU(.clk(Clk_CPU_N),
                 .reset(rst),
                 .MIO_ready(CPU_MIO),
                 .inst_in(spo),
                 .Data_in(Data_read),
                 .mem_w(mem_w),
                 .PC_out(PC_out),
                 .Addr_out(Addr_out),
                 .Data_out(Data_out),
                 .dm_ctrl(dm_ctrl),
                 .CPU_MIO(CPU_MIO),
                 .INT(counter0_OUT)
                 );
    
    Multi_8CH32 U5_Multi_8CH32(.clk(Clk_CPU_N),
                               .rst(rst),
                               .EN(GPIOe0000000_we),
                               .Switch(SW_out[7:5]),
                               .point_in(point_in),
                               .LES(LES),
                               .data0(Peripheral_in),
                               .data1(data1),
                               .data2(spo),
                               .data3(counter_out),
                               .data4(Addr_out),
                               .data5(Data_out),
                               .data6(Cpu_data4bus),
                               .data7(PC_out),
                               .point_out(point_out),
                               .LE_out(LE_out),
                               .Disp_num(Disp_num)
                                );
    
    SSeg7 U6_SSeg7(.clk(clk),
                   .rst(rst),
                   .SW0(SW_out[0]),
                   .flash(clkdiv[10]),
                   .Hexs(Disp_num),
                   .point(point_out),
                   .LES(LE_out),
                   .seg_an(disp_an_o),
                   .seg_sout(disp_seg_o)
                   );
    
    MIO_BUS U4_MIO_BUS(.clk(clk), //
            .rst(rst), //
            .BTN(BTN_out), //
            .SW(SW_out[15:0]), //
            .mem_w(mem_w), //
            .Cpu_data2bus(Data_out), //
            .addr_bus(Addr_out), //
            .ram_data_out(douta), //
            .led_out(LED_out),  //
            .counter_out(counter_out), //
            .counter0_out(counter0_OUT), //
            .counter1_out(counter1_OUT), //
            .counter2_out(counter2_OUT), //
            .Cpu_data4bus(Cpu_data4bus), //
            .ram_data_in(ram_data_in),
            .ram_addr(ram_addr), //
            .GPIOf0000000_we(GPIOf0000000_we), //
            .GPIOe0000000_we(GPIOe0000000_we), //
            .counter_we(counter_we), //
            .Peripheral_in(Peripheral_in) //
            );
endmodule
