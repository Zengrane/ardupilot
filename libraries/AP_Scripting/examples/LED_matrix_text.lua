--[[
Script to control LED strips based on the roll of the aircraft. This is an example to demonstrate
the LED interface for WS2812 LEDs
--]]
-- luacheck: only 0

--[[
for this demo we will use a single strip with 30 LEDs
--]]
local matrix_x = 7
local matrix_y = 7

-- matrix to convert from x y pos to location in the strip
local id = {}
-- because my strips go diagonally to get the led's closer together this is a odd ordering
id[1] = {}
id[1][1] = 21
id[1][2] = 20
id[1][3] = 10
id[1][4] = 9
id[1][5] = 3
id[1][6] = 2
id[1][7] = 0

id[2] = {}
id[2][1] = 33
id[2][2] = 22
id[2][3] = 19
id[2][4] = 11
id[2][5] = 8
id[2][6] = 4
id[2][7] = 1

id[3] = {}
id[3][1] = 34
id[3][2] = 32
id[3][3] = 23
id[3][4] = 18
id[3][5] = 12
id[3][6] = 7
id[3][7] = 5

id[4] = {}
id[4][1] = 42
id[4][2] = 35
id[4][3] = 31
id[4][4] = 24
id[4][5] = 17
id[4][6] = 13
id[4][7] = 6

id[5] = {}
id[5][1] = 43
id[5][2] = 41
id[5][3] = 36
id[5][4] = 30
id[5][5] = 25
id[5][6] = 16
id[5][7] = 14

id[6] = {}
id[6][1] = 47
id[6][2] = 44
id[6][3] = 40
id[6][4] = 37
id[6][5] = 29
id[6][6] = 26
id[6][7] = 15

id[7] = {}
id[7][1] = 48
id[7][2] = 46
id[7][3] = 45
id[7][4] = 39
id[7][5] = 38
id[7][6] = 28
id[7][7] = 27

-- https://github.com/noopkat/oled-font-5x7/blob/master/oled-font-5x7.js
font = {}
font[' '] = {0x00, 0x00, 0x00, 0x00, 0x00} -- // space
font['!'] = {0x00, 0x00, 0x5F, 0x00, 0x00} -- // !
font['"'] = {0x00, 0x07, 0x00, 0x07, 0x00} -- // "
font['#'] = {0x14, 0x7F, 0x14, 0x7F, 0x14} -- // #
font['$'] = {0x24, 0x2A, 0x7F, 0x2A, 0x12} -- // $
font['%'] = {0x23, 0x13, 0x08, 0x64, 0x62} -- // %
font['&'] = {0x36, 0x49, 0x55, 0x22, 0x50} -- // &
font['('] = {0x00, 0x1C, 0x22, 0x41, 0x00} -- // (
font[')'] = {0x00, 0x41, 0x22, 0x1C, 0x00} -- // )
font['*'] = {0x08, 0x2A, 0x1C, 0x2A, 0x08} -- // *
font['+'] = {0x08, 0x08, 0x3E, 0x08, 0x08} -- // +
font[','] = {0x00, 0x50, 0x30, 0x00, 0x00} -- // ,
font['-'] = {0x08, 0x08, 0x08, 0x08, 0x08} -- // -
font['.'] = {0x00, 0x60, 0x60, 0x00, 0x00} -- // .
font['/'] = {0x20, 0x10, 0x08, 0x04, 0x02} -- // /
font['0'] = {0x3E, 0x51, 0x49, 0x45, 0x3E} -- // 0
font['1'] = {0x00, 0x42, 0x7F, 0x40, 0x00} -- // 1
font['2'] = {0x42, 0x61, 0x51, 0x49, 0x46} -- // 2
font['3'] = {0x21, 0x41, 0x45, 0x4B, 0x31} -- // 3
font['4'] = {0x18, 0x14, 0x12, 0x7F, 0x10} -- // 4
font['5'] = {0x27, 0x45, 0x45, 0x45, 0x39} -- // 5
font['6'] = {0x3C, 0x4A, 0x49, 0x49, 0x30} -- // 6
font['7'] = {0x01, 0x71, 0x09, 0x05, 0x03} -- // 7
font['8'] = {0x36, 0x49, 0x49, 0x49, 0x36} -- // 8
font['9'] = {0x06, 0x49, 0x49, 0x29, 0x1E} -- // 9
font[':'] = {0x00, 0x36, 0x36, 0x00, 0x00} -- // :
font[';'] = {0x00, 0x56, 0x36, 0x00, 0x00} -- // ;
font['<'] = {0x00, 0x08, 0x14, 0x22, 0x41} -- // <
font['='] = {0x14, 0x14, 0x14, 0x14, 0x14} -- // =
font['>'] = {0x41, 0x22, 0x14, 0x08, 0x00} -- // >
font['?'] = {0x02, 0x01, 0x51, 0x09, 0x06} -- // ?
font['@'] = {0x32, 0x49, 0x79, 0x41, 0x3E} -- // @
font['A'] = {0x7E, 0x11, 0x11, 0x11, 0x7E} -- // A
font['B'] = {0x7F, 0x49, 0x49, 0x49, 0x36} -- // B
font['C'] = {0x3E, 0x41, 0x41, 0x41, 0x22} -- // C
font['D'] = {0x7F, 0x41, 0x41, 0x22, 0x1C} -- // D
font['E'] = {0x7F, 0x49, 0x49, 0x49, 0x41} -- // E
font['F'] = {0x7F, 0x09, 0x09, 0x01, 0x01} -- // F
font['G'] = {0x3E, 0x41, 0x41, 0x51, 0x32} -- // G
font['H'] = {0x7F, 0x08, 0x08, 0x08, 0x7F} -- // H
font['I'] = {0x00, 0x41, 0x7F, 0x41, 0x00} -- // I
font['J'] = {0x20, 0x40, 0x41, 0x3F, 0x01} -- // J
font['K'] = {0x7F, 0x08, 0x14, 0x22, 0x41} -- // K
font['L'] = {0x7F, 0x40, 0x40, 0x40, 0x40} -- // L
font['M'] = {0x7F, 0x02, 0x04, 0x02, 0x7F} -- // M
font['N'] = {0x7F, 0x04, 0x08, 0x10, 0x7F} -- // N
font['O'] = {0x3E, 0x41, 0x41, 0x41, 0x3E} -- // O
font['P'] = {0x7F, 0x09, 0x09, 0x09, 0x06} -- // P
font['Q'] = {0x3E, 0x41, 0x51, 0x21, 0x5E} -- // Q
font['R'] = {0x7F, 0x09, 0x19, 0x29, 0x46} -- // R
font['S'] = {0x46, 0x49, 0x49, 0x49, 0x31} -- // S
font['T'] = {0x01, 0x01, 0x7F, 0x01, 0x01} -- // T
font['U'] = {0x3F, 0x40, 0x40, 0x40, 0x3F} -- // U
font['V'] = {0x1F, 0x20, 0x40, 0x20, 0x1F} -- // V
font['W'] = {0x7F, 0x20, 0x18, 0x20, 0x7F} -- // W
font['X'] = {0x63, 0x14, 0x08, 0x14, 0x63} -- // X
font['Y'] = {0x03, 0x04, 0x78, 0x04, 0x03} -- // Y
font['Z'] = {0x61, 0x51, 0x49, 0x45, 0x43} -- // Z
font['['] = {0x00, 0x00, 0x7F, 0x41, 0x41} -- // [
font[']'] = {0x41, 0x41, 0x7F, 0x00, 0x00} -- // ]
font['^'] = {0x04, 0x02, 0x01, 0x02, 0x04} -- // ^
font['_'] = {0x40, 0x40, 0x40, 0x40, 0x40} -- // _
font['a'] = {0x20, 0x54, 0x54, 0x54, 0x78} -- // a
font['b'] = {0x7F, 0x48, 0x44, 0x44, 0x38} -- // b
font['c'] = {0x38, 0x44, 0x44, 0x44, 0x20} -- // c
font['d'] = {0x38, 0x44, 0x44, 0x48, 0x7F} -- // d
font['e'] = {0x38, 0x54, 0x54, 0x54, 0x18} -- // e
font['f'] = {0x08, 0x7E, 0x09, 0x01, 0x02} -- // f
font['g'] = {0x08, 0x14, 0x54, 0x54, 0x3C} -- // g
font['h'] = {0x7F, 0x08, 0x04, 0x04, 0x78} -- // h
font['i'] = {0x00, 0x44, 0x7D, 0x40, 0x00} -- // i
font['j'] = {0x20, 0x40, 0x44, 0x3D, 0x00} -- // j
font['k'] = {0x00, 0x7F, 0x10, 0x28, 0x44} -- // k
font['l'] = {0x00, 0x41, 0x7F, 0x40, 0x00} -- // l
font['m'] = {0x7C, 0x04, 0x18, 0x04, 0x78} -- // m
font['n'] = {0x7C, 0x08, 0x04, 0x04, 0x78} -- // n
font['o'] = {0x38, 0x44, 0x44, 0x44, 0x38} -- // o
font['p'] = {0x7C, 0x14, 0x14, 0x14, 0x08} -- // p
font['q'] = {0x08, 0x14, 0x14, 0x18, 0x7C} -- // q
font['r'] = {0x7C, 0x08, 0x04, 0x04, 0x08} -- // r
font['s'] = {0x48, 0x54, 0x54, 0x54, 0x20} -- // s
font['t'] = {0x04, 0x3F, 0x44, 0x40, 0x20} -- // t
font['u'] = {0x3C, 0x40, 0x40, 0x20, 0x7C} -- // u
font['v'] = {0x1C, 0x20, 0x40, 0x20, 0x1C} -- // v
font['w'] = {0x3C, 0x40, 0x30, 0x40, 0x3C} -- // w
font['x'] = {0x44, 0x28, 0x10, 0x28, 0x44} -- // x
font['y'] = {0x0C, 0x50, 0x50, 0x50, 0x3C} -- // y
font['z'] = {0x44, 0x64, 0x54, 0x4C, 0x44} -- // z
font['{'] = {0x00, 0x08, 0x36, 0x41, 0x00} -- // {
font['|'] = {0x00, 0x00, 0x7F, 0x00, 0x00} -- // |
font['}'] = {0x00, 0x41, 0x36, 0x08, 0x00} -- // }

--[[
 use SERVOn_FUNCTION 94 for LED. We can control up to 16 separate strips of LEDs
 by putting them on different channels
--]]
local chan = SRV_Channels:find_channel(94)

if not chan then
    gcs:send_text(6, "LEDs: channel not set")
    return
end

-- find_channel returns 0 to 15, convert to 1 to 16
chan = chan + 1

gcs:send_text(6, "LEDs: chan=" .. tostring(chan))

-- initialisation code
--serialLED:set_num_neopixel(chan,  matrix_x * matrix_y)
serialLED:set_num_profiled(chan,  matrix_x * matrix_y)

local offset = 8;
local text_string = "ArduPilot"

local function display_char(char,r,g,b,offset_in)
local char_offset = 0
if offset_in then
    char_offset = offset_in
end
if char_offset > matrix_x then
    return
end
if char_offset < 1 - 5 then
    return
end

local i
local j

for i = 1, 5 do
    local x_index = i + char_offset
    if x_index >= 1 and x_index <= matrix_x then
        local font_colum = font[char][i]
        for j = 1, 7 do
            if (font_colum & 1) == 1 then
                serialLED:set_RGB(chan, id[j][x_index], r, g, b)
            end
            font_colum = font_colum >> 1
        end
    end
end

end

local function display_string(string,r,g,b,offset_in)
local str_offset = 0
local i
for i = 1, string:len() do
    display_char(string:sub(i,i),r,g,b,str_offset + offset_in)
    str_offset = str_offset + 6
end
end

function update_LEDs()

  serialLED:set_RGB(chan, -1, 0, 0, 0)

  display_string(text_string,100,0,0,offset)

  serialLED:send(chan)

  offset = offset - 1

 -- scroll until it is off the left edge
  if offset < -text_string:len()*6 then
    -- start with the stuff off the right edge of the display
    offset = 8

    text_string = tostring(math.floor(math.deg(ahrs:get_yaw())))
  end

  return update_LEDs, 100
end

return update_LEDs, 1000

