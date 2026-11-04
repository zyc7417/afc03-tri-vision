`timescale 1ns/1ps
module tb_tone;
 parameter W=16,H=12,SEED=1;
 localparam N=W*H,WORDS=(W+3)/4,BEATS=WORDS*H;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,sv=0,rv=0,ss=0,se=0,sf=0,rs=0,re=0,rf=0,mr=0;
 reg [127:0] sd=0,rd=0;reg [3:0] sk=0,rk=0;
 wire [3:0] sr,rr,lr,mv,err,ms,me,mf;
 wire [127:0] md[0:3];wire [3:0]mk[0:3];
 histogram_equalize4_old #(.WIDTH(W),.HEIGHT(H)) original(
 .clk(clk),.rst_n(rst_n),.s_valid(sv),.s_ready(sr[0]),.s_data(sd),.s_keep(sk),.s_sof(ss),.s_eol(se),.s_eof(sf),
 .lut_ready(lr[0]),.r_valid(rv),.r_ready(rr[0]),.r_data(rd),.r_keep(rk),.r_sof(rs),.r_eol(re),.r_eof(rf),
 .m_valid(mv[0]),.m_ready(mr),.m_data(md[0]),.m_keep(mk[0]),.m_sof(ms[0]),.m_eol(me[0]),.m_eof(mf[0]),.protocol_error(err[0]));
 genvar g;generate for(g=0;g<3;g=g+1)begin:d
 histogram_equalize4 #(.WIDTH(W),.HEIGHT(H),.HE_MODE(g)) u(
 .clk(clk),.rst_n(rst_n),.s_valid(sv),.s_ready(sr[g+1]),.s_data(sd),.s_keep(sk),.s_sof(ss),.s_eol(se),.s_eof(sf),
 .lut_ready(lr[g+1]),.r_valid(rv),.r_ready(rr[g+1]),.r_data(rd),.r_keep(rk),.r_sof(rs),.r_eol(re),.r_eof(rf),
 .m_valid(mv[g+1]),.m_ready(mr),.m_data(md[g+1]),.m_keep(mk[g+1]),.m_sof(ms[g+1]),.m_eol(me[g+1]),.m_eof(mf[g+1]),.protocol_error(err[g+1]));
 end endgenerate
 integer pixels[0:N-1],hist[0:255],expected[0:255],bounded[0:255];
 integer cycle=0,outputs=0,frames=0,stalls=0,observed_stalls=0,checks=0;
 integer frame_case=0,checking=0,x,j,k,m,acc,total,cmin,mix,lo,hi,v,seen;
 reg [31:0] random_state=SEED;reg[127:0]held[0:3];reg[3:0]held_keep[0:3];reg[3:0]held_sof,held_eol,held_eof;reg held_valid=0;
 function [31:0] rand_next(input[31:0]a);begin rand_next={a[30:0],a[31]^a[21]^a[1]^a[0]};end endfunction
 always @(negedge clk) begin
  random_state=rand_next(random_state);
  mr=rst_n&&(random_state[2:0]!=0)&&(cycle%31<23);
 end
 always @(posedge clk) begin
  cycle=cycle+1;if(cycle>2000000)$fatal(1,"TIMEOUT");
  if(rst_n)begin
   if(sr!=={4{sr[0]}}||rr!=={4{rr[0]}}||lr!=={4{lr[0]}}||mv!=={4{mv[0]}}||err!=={4{err[0]}})$fatal(1,"HANDSHAKE_MITER");
   if(mv[0])begin
    if(md[0]!==md[2]||mk[0]!==mk[2]||ms[0]!==ms[2]||me[0]!==me[2]||mf[0]!==mf[2])$fatal(1,"ORIGINAL_MODE_NOT_CYCLE_EQUIVALENT");
   end
   if(held_valid)begin
    for(m=0;m<4;m=m+1)if(!mv[m]||md[m]!==held[m]||mk[m]!==held_keep[m]||ms[m]!==held_sof[m]||me[m]!==held_eol[m]||mf[m]!==held_eof[m])$fatal(1,"OUTPUT_CHANGED_UNDER_BACKPRESSURE");
    observed_stalls=observed_stalls+1;
   end
   if(mv[0]&&mr&&checking)begin
    x=(outputs%WORDS)*4;j=(outputs/WORDS)*W+x;
    if(ms!=={4{outputs==0}}||me!=={4{outputs%WORDS==WORDS-1}}||mf!=={4{outputs==BEATS-1}})$fatal(1,"SIDEBAND");
    for(k=0;k<4;k=k+1)begin
     if(x+k<W)begin
      for(m=0;m<4;m=m+1)begin
       v=m==1?pixels[j+k]:(m==3?bounded[pixels[j+k]]:expected[pixels[j+k]]);
       if(md[m][32*k+:32]!=={8'd0,8'(v),8'(v),8'(v)}||!mk[m][k])$fatal(1,"PIXEL case=%0d word=%0d lane=%0d mode=%0d actual=%h expected=%0d",frame_case,outputs,k,m,md[m][32*k+:32],v);
      end
     end else for(m=0;m<4;m=m+1)if(md[m][32*k+:32]!==0||mk[m][k])$fatal(1,"TAIL_KEEP");
    end
    outputs=outputs+1;
   end
  end
  held_valid=rst_n&&mv[0]&&!mr;
  for(m=0;m<4;m=m+1)begin held[m]=md[m];held_keep[m]=mk[m];end
  held_sof=ms;held_eol=me;held_eof=mf;
 end
 task reset_all;begin
  @(negedge clk);rst_n=0;sv=0;rv=0;checking=0;held_valid=0;
  repeat(4)@(negedge clk);rst_n=1;
 end endtask
 task scene(input integer which);integer i,a,b,c,den;begin
  frame_case=which;outputs=0;
  for(i=0;i<256;i=i+1)hist[i]=0;
  for(i=0;i<N;i=i+1)begin
   case(which)
    0:pixels[i]=0;
    1:pixels[i]=150;
    2:pixels[i]=255;
    3:pixels[i]=i>=N*3/4?0:150;
    4:pixels[i]=(i*4/N)*40;
    5:pixels[i]=i%256;
    6:pixels[i]=i==N/2?255:120;
    7:pixels[i]=i==N/2?0:190;
    8:pixels[i]=i%2?50:200;
    default:begin random_state=rand_next(random_state);pixels[i]=random_state[7:0];end
   endcase
   hist[pixels[i]]=hist[pixels[i]]+1;
  end
  a=0;b=0;c=0;for(i=0;i<256;i=i+1)if(hist[i]!=0&&!c)begin b=hist[i];c=1;end
  den=N-b;
  for(i=0;i<256;i=i+1)begin
   a=a+hist[i];expected[i]=den==0?i:((a<=b)?0:((a-b)*255+den/2)/den);
   mix=(3*i+expected[i]+2)/4;lo=i<16?0:i-16;hi=i>239?255:i+16;
   bounded[i]=mix<lo?lo:(mix>hi?hi:mix);
   if(i>0&&bounded[i]<bounded[i-1])$fatal(1,"ORACLE_NOT_MONOTONE");
  end
  if(bounded[0]!=0)$fatal(1,"BLACK_CHANGED");
 end endtask
 task send_beat(input integer replay,input integer b,input integer bad);integer col,row,n,i;reg[127:0]data;reg[3:0]keep;begin
  col=(b%WORDS)*4;row=b/WORDS;data=0;keep=0;
  for(i=0;i<4;i=i+1)if(col+i<W)begin data[i*32+:32]={8'd0,8'(pixels[row*W+col+i]),8'(pixels[row*W+col+i]),8'(pixels[row*W+col+i])};keep[i]=1;end
  n=random_state[1:0];repeat(n)@(negedge clk);
  @(negedge clk);
  if(replay)begin rv=1;rd=data;rk=(bad==2)?0:keep;rs=b==0;re=b%WORDS==WORDS-1;rf=(b==BEATS-1)^(bad==1);end
  else begin sv=1;sd=data;sk=(bad==2)?0:keep;ss=b==0;se=b%WORDS==WORDS-1;sf=(b==BEATS-1)^(bad==1);end
  @(posedge clk);while(replay?!rr[0]:!sr[0])@(posedge clk);
  @(negedge clk);if(replay)rv=0;else sv=0;
 end endtask
 task complete_frame(input integer which);integer b,i;begin
  scene(which);wait(sr[0]);for(b=0;b<BEATS;b=b+1)send_beat(0,b,0);
  wait(lr[0]);@(negedge clk);
  for(i=0;i<256;i=i+1)begin
   if(original.lut0[i]!==8'(expected[i])||d[0].u.lut0[i]!==8'(i)||d[1].u.lut0[i]!==8'(expected[i])||d[2].u.lut0[i]!==8'(bounded[i]))$fatal(1,"LUT_ORACLE idx=%0d",i);
  end
  checking=1;for(b=0;b<BEATS;b=b+1)send_beat(1,b,0);
  wait(outputs==BEATS);@(negedge clk);checking=0;frames=frames+1;
  wait(sr[0]);repeat(3)@(negedge clk);if(mv!=0)$fatal(1,"DUPLICATE_TAIL");
 end endtask
 integer aa,bb,got,expect_v,f;
 initial begin
  // Exhaust all input/HE-value pairs using the actual synthesized function.
  for(aa=0;aa<256;aa=aa+1)for(bb=0;bb<256;bb=bb+1)begin
   expect_v=(3*aa+bb+2)/4;if(expect_v<aa-16)expect_v=aa-16;if(expect_v>aa+16)expect_v=aa+16;
   got=d[2].u.tone_value(8'(aa),8'(bb));
   if(got!=expect_v||got<0||got>255||got-aa>16||aa-got>16)$fatal(1,"EXHAUSTIVE_TONE");checks=checks+1;
  end
  reset_all();for(f=0;f<14;f=f+1)complete_frame(f);
  // Bad statistics/replay EOF and keep reject; reset returns to a clean frame.
  for(f=1;f<=2;f=f+1)begin
   scene(10);send_beat(0,0,f);repeat(3)@(negedge clk);if(err!==4'hf)$fatal(1,"BAD_STATS_NOT_REJECTED");reset_all();complete_frame(3);
   scene(10);for(aa=0;aa<BEATS;aa=aa+1)send_beat(0,aa,0);wait(lr[0]);send_beat(1,0,f);repeat(3)@(negedge clk);if(err!==4'hf)$fatal(1,"BAD_REPLAY_NOT_REJECTED");reset_all();complete_frame(8);
  end
  scene(10);send_beat(0,0,0);reset_all();complete_frame(11);
  scene(10);for(aa=0;aa<BEATS;aa=aa+1)send_beat(0,aa,0);wait(lr[0]);send_beat(1,0,0);reset_all();complete_frame(12);
  if(observed_stalls==0)$fatal(1,"NO_BACKPRESSURE_COVERED");
  $display("TONE_PASS W=%0d H=%0d SEED=%0d frames=%0d exhaustive=%0d stalled_cycles=%0d cycles=%0d",W,H,SEED,frames,checks,observed_stalls,cycle);$finish;
 end
endmodule
