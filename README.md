# Image_converter
## 1. Introduction

본 프로젝트는 Verilog HDL을 이용하여 이미지 프로세싱 파이프라인 중 Sobel와 laplacian(8방향향) 필터를 직접 구현하고, 테스트벤치를 통해 `.hex` 형식의 입력 파일을 받아 `.ppm` 이미지로 출력하는 시스템을 구축하는 것을 목표로 하였다. 주요 처리 과정은 다음과 같다:

- `img_buf`를 통해 이미지 데이터를 입력
- RGB → Grayscale 변환
- 3x3 커널 기반 Sobel, laplacian Gx/Gy 연산
- Magnitude 계산 및 스케일링
- `o_data_r/g/b`로 결과 출력 후 `.ppm` 생성

  <img width="1360" height="776" alt="Image" src="https://github.com/user-attachments/assets/d6b4cee4-b004-42da-9f65-920ab6e97b40" />


## 2-1. Code Overview and Explanation

### ISP_sobel 코드
module ISP_sobel #(
    parameter BW = 8,    //비트폭, 2^8 (0~255)
    parameter WIDTH = 640  //line buffer 크기, 가로해상도
) (
    input                       clk,
    input                       reset,

    input                       i_frame_start,
    input                       i_valid,
    input                       i_h_first,
    input                       i_h_last,
    input                       i_v_last,
    input       [BW-1:0]        i_data_r,
    input       [BW-1:0]        i_data_g,
    input       [BW-1:0]        i_data_b,

    output                      o_valid,
    output                      o_h_first,
    output                      o_h_last,
    output                      o_v_last,
    output      [BW-1:0]        o_data_r,
    output      [BW-1:0]        o_data_g,
    output      [BW-1:0]        o_data_b
);

    `define macro_control_ff(out, in, clk) \
        always @(posedge clk or negedge reset) \
        begin \
            if (!reset)  out <= 1'b0; \
            else if(enable) out <= in; \         //reset=0이면 out0, enable=1이면 out=in
        end

    //================ Control Delay =================
    reg valid_1d, valid_2d, valid_3d, valid_4d, valid_5d, valid_6d;
    reg h_first_1d, h_first_2d, h_first_3d, h_first_4d, h_first_5d, h_first_6d;
    reg h_last_1d, h_last_2d, h_last_3d, h_last_4d, h_last_5d, h_last_6d;
    reg v_last_1d, v_last_2d, v_last_3d, v_last_4d, v_last_5d, v_last_6d;      

    wire gclk;
    wire enable = (i_valid | valid_1d | valid_2d | valid_3d | valid_4d | valid_5d | valid_6d) | !reset;   //nand
    clk_gate clk_gate_0 (.gclk(gclk), .enable(enable), .clk(clk));

    `macro_control_ff(valid_1d, i_valid, clk)
    `macro_control_ff(valid_2d, valid_1d, gclk)
    `macro_control_ff(valid_3d, valid_2d, gclk)
    `macro_control_ff(valid_4d, valid_3d, gclk)
    `macro_control_ff(valid_5d, valid_4d, gclk)
    `macro_control_ff(valid_6d, valid_5d, gclk)

    `macro_control_ff(h_first_1d, i_h_first, clk)
    `macro_control_ff(h_first_2d, h_first_1d, gclk)
    `macro_control_ff(h_first_3d, h_first_2d, gclk)
    `macro_control_ff(h_first_4d, h_first_3d, gclk)
    `macro_control_ff(h_first_5d, h_first_4d, gclk)
    `macro_control_ff(h_first_6d, h_first_5d, gclk)

    `macro_control_ff(h_last_1d, i_h_last, clk)
    `macro_control_ff(h_last_2d, h_last_1d, gclk)
    `macro_control_ff(h_last_3d, h_last_2d, gclk)
    `macro_control_ff(h_last_4d, h_last_3d, gclk)
    `macro_control_ff(h_last_5d, h_last_4d, gclk)
    `macro_control_ff(h_last_6d, h_last_5d, gclk)

    `macro_control_ff(v_last_1d, i_v_last, clk)
    `macro_control_ff(v_last_2d, v_last_1d, gclk)
    `macro_control_ff(v_last_3d, v_last_2d, gclk)
    `macro_control_ff(v_last_4d, v_last_3d, gclk)
    `macro_control_ff(v_last_5d, v_last_4d, gclk)
    `macro_control_ff(v_last_6d, v_last_5d, gclk)

    //================ Grayscale Pipeline ================
    wire [BW-1:0] gray_pixel;
    reg  [BW-1:0] gray_pixel_d1, gray_pixel_d2, gray_pixel_d3;   //한클럭식 지연시켜 수평으로 3줄 만듦

    assign gray_pixel = (i_data_r * 77 + i_data_g * 150 + i_data_b * 29) >> 8;    //근사값으로 흑백처

    always @(posedge gclk or negedge reset) begin
        if(!reset) begin
            gray_pixel_d1 <= 0; gray_pixel_d2 <= 0; gray_pixel_d3 <= 0;
        end else begin
            if (i_valid)     gray_pixel_d1 <= gray_pixel;
            if (valid_1d)    gray_pixel_d2 <= gray_pixel_d1;
            if (valid_2d)    gray_pixel_d3 <= gray_pixel_d2;
        end
    end

    //================ Line Buffers ================
    reg [BW-1:0] line_buffer1 [0:WIDTH-1];   //이전행 ,가로 인덱스 (0 ~ WIDTH-1)
    reg [BW-1:0] line_buffer2 [0:WIDTH-1];   //그 이전행 , 현재행은 gray_pixel_d3
    reg [$clog2(WIDTH)-1:0] addr;
    wire [BW-1:0] line1_data, line2_data;

    always @(posedge gclk or negedge reset) begin   //리셋 → 0으로 초기화. valid_3d일 때만 주소 증가. 
        if (!reset) addr <= 0;
        else if (valid_3d) begin
            if (h_first_3d) addr <= 1;    //한 줄의 첫 픽셀(h_first_3d)이면 addr을 1로 초기화. 나머지 경우에는 addr을 +1 하면서 순차적으로 가로줄을 스캔.
            else           addr <= addr + 1;
        end
    end

    always @(posedge gclk) begin 
        if (valid_3d) begin
            line_buffer2[addr] <= line_buffer1[addr];    //새로운 픽셀이 들어올 때마다 line_buffer1에 기록. 기존 line_buffer1 값은 line_buffer2로 밀려남.
            line_buffer1[addr] <= gray_pixel_d3;         //3d 이후부터 유
        end
    end

    assign line1_data = line_buffer1[addr];
    assign line2_data = line_buffer2[addr];

    //================ Window =================
    reg [BW-1:0] p11, p12, p13;
    reg [BW-1:0] p21, p22, p23;
    reg [BW-1:0] p31, p32, p33;

    always @(posedge gclk or negedge reset) begin
        if(!reset) begin
            p11<=0; p12<=0; p13<=0;
            p21<=0; p22<=0; p23<=0;
            p31<=0; p32<=0; p33<=0;
        end else if (valid_4d) begin      //쉬프트하면서 오른쪽으로 새 픽셀 삽입
            p31 <= p32; p32 <= p33; p33 <= gray_pixel_d3;
            p21 <= p22; p22 <= p23; p23 <= line1_data;
            p11 <= p12; p12 <= p13; p13 <= line2_data;
        end
    end

    //================ Sobel =================
    wire signed [BW+2:0] Gx = (p13 + (p23 << 1) + p33) - (p11 + (p21 << 1) + p31);     //마스크를 코드로 변환함
    wire signed [BW+2:0] Gy = (p31 + (p32 << 1) + p33) - (p11 + (p12 << 1) + p13);     //부호 생김
    reg signed [BW+2:0] Gx_d1, Gy_d1;

    always @(posedge gclk or negedge reset) begin
        if(!reset) begin Gx_d1 <= 0; Gy_d1 <= 0; end
        else if (valid_5d) begin Gx_d1 <= Gx; Gy_d1 <= Gy; end
    end

    //================ Magnitude =================
	wire [BW+2:0] abs_Gx = Gx_d1[BW+2] ? -Gx_d1 : Gx_d1;     //절댓값
	wire [BW+2:0] abs_Gy = Gy_d1[BW+2] ? -Gy_d1 : Gy_d1;
	wire [BW+3:0] G_temp = abs_Gx + abs_Gy;
	wire [BW+3:0] G_scaled = G_temp >> 2;     //노멀라이즈 출력 포화 방지

	reg [BW-1:0] edge_magnitude;

	always @(posedge gclk or negedge reset) begin
		if (!reset) edge_magnitude <= 0;
		else if (valid_6d) edge_magnitude <= G_scaled[BW-1:0];
	end

    assign o_valid   = valid_6d;
    assign o_h_first = h_first_6d;
    assign o_h_last  = h_last_6d;
    assign o_v_last  = v_last_6d;
    assign o_data_r  = edge_magnitude;
    assign o_data_g  = edge_magnitude;
    assign o_data_b  = edge_magnitude;

endmodule

## 2-2. Results
결과 이미지는 다음과 같다

![Image](https://github.com/user-attachments/assets/c7e77d53-2de3-441e-83a9-b434fd65e3a2)

## 3-1. Code Overview and Explanation

### ISP_laplacian 코드
module ISP_laplacian #(
    parameter BW = 8,
    parameter WIDTH = 640
) (
    input                       clk,
    input                       reset,

    input                       i_frame_start,
    input                       i_valid,
    input                       i_h_first,
    input                       i_h_last,
    input                       i_v_last,
    input       [BW-1:0]        i_data_r,
    input       [BW-1:0]        i_data_g,
    input       [BW-1:0]        i_data_b,

    output                      o_valid,
    output                      o_h_first,
    output                      o_h_last,
    output                      o_v_last,
    output      [BW-1:0]        o_data_r,
    output      [BW-1:0]        o_data_g,
    output      [BW-1:0]        o_data_b
);

    `define macro_control_ff(out, in, clk) \
        always @(posedge clk or negedge reset) \
        begin \
            if (!reset)  out <= 1'b0; \
            else if(enable) out <= in; \
        end

    //================ Control Delay =================
    reg valid_1d, valid_2d, valid_3d, valid_4d, valid_5d, valid_6d;
    reg h_first_1d, h_first_2d, h_first_3d, h_first_4d, h_first_5d, h_first_6d;
    reg h_last_1d, h_last_2d, h_last_3d, h_last_4d, h_last_5d, h_last_6d;
    reg v_last_1d, v_last_2d, v_last_3d, v_last_4d, v_last_5d, v_last_6d;

    wire gclk;
    wire enable = (i_valid | valid_1d | valid_2d | valid_3d | valid_4d | valid_5d | valid_6d) | !reset;
    clk_gate clk_gate_0 (.gclk(gclk), .enable(enable), .clk(clk));

    `macro_control_ff(valid_1d, i_valid, clk)
    `macro_control_ff(valid_2d, valid_1d, gclk)
    `macro_control_ff(valid_3d, valid_2d, gclk)
    `macro_control_ff(valid_4d, valid_3d, gclk)
    `macro_control_ff(valid_5d, valid_4d, gclk)
    `macro_control_ff(valid_6d, valid_5d, gclk)

    `macro_control_ff(h_first_1d, i_h_first, clk)
    `macro_control_ff(h_first_2d, h_first_1d, gclk)
    `macro_control_ff(h_first_3d, h_first_2d, gclk)
    `macro_control_ff(h_first_4d, h_first_3d, gclk)
    `macro_control_ff(h_first_5d, h_first_4d, gclk)
    `macro_control_ff(h_first_6d, h_first_5d, gclk)

    `macro_control_ff(h_last_1d, i_h_last, clk)
    `macro_control_ff(h_last_2d, h_last_1d, gclk)
    `macro_control_ff(h_last_3d, h_last_2d, gclk)
    `macro_control_ff(h_last_4d, h_last_3d, gclk)
    `macro_control_ff(h_last_5d, h_last_4d, gclk)
    `macro_control_ff(h_last_6d, h_last_5d, gclk)

    `macro_control_ff(v_last_1d, i_v_last, clk)
    `macro_control_ff(v_last_2d, v_last_1d, gclk)
    `macro_control_ff(v_last_3d, v_last_2d, gclk)
    `macro_control_ff(v_last_4d, v_last_3d, gclk)
    `macro_control_ff(v_last_5d, v_last_4d, gclk)
    `macro_control_ff(v_last_6d, v_last_5d, gclk)

    //================ Grayscale Pipeline ================
    wire [BW-1:0] gray_pixel;
    reg  [BW-1:0] gray_pixel_d1, gray_pixel_d2, gray_pixel_d3;

    assign gray_pixel = (i_data_r * 77 + i_data_g * 150 + i_data_b * 29) >> 8;

    always @(posedge gclk or negedge reset) begin
        if(!reset) begin
            gray_pixel_d1 <= 0; gray_pixel_d2 <= 0; gray_pixel_d3 <= 0;
        end else begin
            if (i_valid)     gray_pixel_d1 <= gray_pixel;
            if (valid_1d)    gray_pixel_d2 <= gray_pixel_d1;
            if (valid_2d)    gray_pixel_d3 <= gray_pixel_d2;
        end
    end

    //================ Line Buffers ================
    reg [BW-1:0] line_buffer1 [0:WIDTH-1];
    reg [BW-1:0] line_buffer2 [0:WIDTH-1];
    reg [$clog2(WIDTH)-1:0] addr;
    wire [BW-1:0] line1_data, line2_data;

    always @(posedge gclk or negedge reset) begin
        if (!reset) addr <= 0;
        else if (valid_3d) begin
            if (h_first_3d) addr <= 1;
            else           addr <= addr + 1;
        end
    end

    always @(posedge gclk) begin
        if (valid_3d) begin
            line_buffer2[addr] <= line_buffer1[addr];
            line_buffer1[addr] <= gray_pixel_d3;
        end
    end

    assign line1_data = line_buffer1[addr];
    assign line2_data = line_buffer2[addr];

    //================ Window =================
    reg [BW-1:0] p11, p12, p13;
    reg [BW-1:0] p21, p22, p23;
    reg [BW-1:0] p31, p32, p33;

    always @(posedge gclk or negedge reset) begin
        if(!reset) begin
            p11<=0; p12<=0; p13<=0;
            p21<=0; p22<=0; p23<=0;
            p31<=0; p32<=0; p33<=0;
        end else if (valid_4d) begin
            p31 <= p32; p32 <= p33; p33 <= gray_pixel_d3;
            p21 <= p22; p22 <= p23; p23 <= line1_data;
            p11 <= p12; p12 <= p13; p13 <= line2_data;
        end
    end

    //================ laplacian =================
    wire signed [BW+3:0] Gx = ($signed({1'b0,p22}) <<< 3)     //p22 <<< 3 는 unsign이고 p11~p33역시 unsign 하지만 언사인드끼리 연산해도 변수(Gx)가 사인드니까 언사인드를 사인드 시켜줘야함.
   - ($signed({1'b0,p11}) + $signed({1'b0,p12}) + $signed({1'b0,p13})
   +  $signed({1'b0,p21}) + $signed({1'b0,p23})
   +  $signed({1'b0,p31}) + $signed({1'b0,p32}) + $signed({1'b0,p33}));

	reg signed [BW+3:0] Gx_d1;

	
    always @(posedge gclk or negedge reset) begin
        if(!reset) begin Gx_d1 <= 0; end
        else if (valid_5d) begin Gx_d1 <= Gx; end
    end

    //================ Magnitude =================
	wire [BW+3:0] abs_Gx = Gx_d1[BW+2] ? -Gx_d1 : Gx_d1; 
	wire [BW+3:0] G_temp = abs_Gx;
	wire [BW+3:0] G_scaled = G_temp >> 2;  

	reg [BW-1:0] edge_magnitude;

	
	 //================ saturation =================	
	always @(posedge gclk or negedge reset) begin
    if (!reset) edge_magnitude <= 0;
    else if (valid_6d) begin
        if (G_scaled > 8'd255)
            edge_magnitude <= 8'd255;
        else
            edge_magnitude <= G_scaled[BW-1:0];
    end
end

	
	
    assign o_valid   = valid_6d;
    assign o_h_first = h_first_6d;
    assign o_h_last  = h_last_6d;
    assign o_v_last  = v_last_6d;
    assign o_data_r  = edge_magnitude;
    assign o_data_g  = edge_magnitude;
    assign o_data_b  = edge_magnitude;

endmodule

## 3-2. Results
![lena_out (2)](https://github.com/user-attachments/assets/46dfabf0-c779-49ec-88ee-80359b1fe2a2)


# 4. Comparative Analysis

## 🔹 Sobel filter
- `Gx`, `Gy` 방향 각각 gradient 계산
- 절댓값 후 합산하여 magnitude 추출  
  (예: `|Gx| + |Gy|`)
- 결과를 RGB 채널에 동일하게 출력하여 Grayscale edge map 생성

## 🔹 Laplacian filter
- 3×3 커널: `8 × p22 − 주변 8픽셀의 합`
- 모든 입력 픽셀을 `$signed({1'b0, pXX})` 방식으로 sign-extend
- 연산 결과에 절댓값 + shift 적용 (스케일링)
- `if (value > 255) value = 255;` 방식으로 포화 클리핑 처리
- RGB 3채널 동일 출력 (grayscale edge image)

---

## Comparison of Simulation Results

| 항목               | Sobel 필터 결과                              | Laplacian 필터 결과                           |
|--------------------|----------------------------------------------|-----------------------------------------------|
| 윤곽선 두께         | 굵고 부드러운 외곽선                          | 얇고 날카로운 외곽선                            |
| 디테일 표현         | 주요 윤곽 위주로 강조                         | 작은 텍스처나 미세한 변화까지 반응               |
| 노이즈 반응         | 노이즈는 무시되거나 흐리게 나타남             | 노이즈 포함 고주파 성분도 edge처럼 표현됨        |
| 텍스트/선 표현      | 두께감 있는 안정된 edge                      | 날카롭고 얇은 선 표현 가능                      |

---

## 5. Conclusion and Future Work

본 프로젝트는 Verilog로 구현된 소벨 및 라플라시안 필터의 비교 분석을 수행했다. 소벨 필터는 노이즈에 대해 덜 받는 특성으로 방향성이 명확한 엣지를 검출하는 데 매우 효과적이었으며, 이는 일반적인 엣지 검출 및 전처리 작업에 적합함을 보여준다. 반면, 2차 미분 연산자인 라플라시안 필터는 미세한 디테일과 고주파 성분에 대해 더 뛰어난 민감도를 보였다. 이는 노이즈에 대한 민감도가 증가하는 단점이 있지만, 미세한 질감을 강조해야 하는 경우에서 더 효과적임을 의미한다.   
Verilog를 이용한 직접적인 하드웨어 구현은 소프트웨어와는 크게 다른 하드웨어 고유의 설계 고려 사항에 대한 중요한 통찰을 제공했다. 주요 과제로는 신중한 비트 폭 할당을 통한 산술 정밀도 관리와 부호 확장의 올바른 처리가 있었다. 또한, 비용이 높은 곱셈기 대신 비트 단위 시프트 연산을 활용하는 것과 같은 하드웨어 자원 최적화를 위한 전략적 결정이 요구되었다. 오버플로우를 방지하기 위한 출력 스케일링 및 포화 연산과 같은 후처리 로직의 필요성 또한 명확히 확인되었다.
본 연구를 통해 소벨 필터는 노이즈에 강하지만 상대적으로 윤곽선이 두껍게 표현되고, 라플라시안 필터는 윤곽선이 날카롭지만 노이즈에 취약한 상충 관계를 확인했다. 실제 시스템에 적용하기 위해서는 두 필터의 장점을 모두 취할 필요가 있으며, 이에 대한 해결책으로 LoG(Laplacian of Gaussian) 필터를 이용한다. LoG 구조는 먼저 가우시안 블러를 통해 노이즈를 제거하여 소벨 필터와 같은 안정성을 확보한 뒤, 라플라시안 연산으로 날카로운 엣지를 검출한다. 이 방식을 통해 노이즈 억제(디노이즈)와 정밀한 윤곽선 검출이라는 두 가지 목표를 동시에 만족시키는 고성능 시스템을 구현할 수 있을 것으로 기대된다.
