// Copyright (C) 2022-2023, Scott Eric Gilbert.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0.  If a copy of the MPL was not distributed with
// this file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// This Source Code Form is "Incompatible With Secondary Licenses".
#pragma once

#include <math.h>
#include <stdint.h>
#include <limits.h>

#include <algorithm>
#include <unordered_map>

namespace monomath {

    struct RGB { uint8_t r, g, b; };
    static inline RGB blend(RGB dst, RGB src, long amt) {
        long inv = 256 - amt;
        return RGB {
            uint8_t((dst.r*inv + src.r*amt)>>8),
            uint8_t((dst.g*inv + src.g*amt)>>8),
            uint8_t((dst.b*inv + src.b*amt)>>8)
        };
    }

    struct Rect { long l, t, r, b; };

    struct Options {
        double width = 16;     // glyph width in pixels
        double height = 32;    // glyph height in pixels
        double weight = 1.0;   // light: 0.5, very heavy 4.0
        double spacing = 0.5;  // horizontal spacing of chars
        double leading = 0.25; // vertical spacing of lines
        double oblique = 0.0;  // 0.5 for faux italics
        double degrees = 0.0;  // counter clockwise rotation
        // Restrict drawing to { left, top, right, and bottom }
        Rect clip = { LONG_MIN, LONG_MIN, LONG_MAX, LONG_MAX };
    };

    const long GLYPH_XRANGE = 16;
    const long GLYPH_YRANGE = 32;

    const long FIELD_MARGIN = 8;
    const long FIELD_XRANGE = 32;
    const long FIELD_YRANGE = 64;
    const long FIELD_WIDTH  = FIELD_XRANGE + 2*FIELD_MARGIN + 1;
    const long FIELD_HEIGHT = FIELD_YRANGE + 2*FIELD_MARGIN + 1;

    struct Field { float data[FIELD_HEIGHT][FIELD_WIDTH]; };

    struct Font {
        inline Font();

        template<class dstpixel, class srcpixel, class chartype>
        void draw(
            dstpixel* pixmap, long width, long height, srcpixel color,
            double xx, double yy, const chartype* text,
            const Options& opts=Options(), long stride=-1
        );

        private:
            std::unordered_map<char32_t, long> code_to_index;
            std::unordered_map<long, Field> index_to_field;

            const Field& lookup_field(char32_t code);
    };

    static inline Rect intersect(const Rect& pp, const Rect& qq) {
        long ll = std::max(std::min(pp.l, pp.r), std::min(qq.l, qq.r));
        long rr = std::min(std::max(pp.l, pp.r), std::max(qq.l, qq.r));
        long tt = std::max(std::min(pp.t, pp.b), std::min(qq.t, qq.b));
        long bb = std::min(std::max(pp.t, pp.b), std::max(qq.t, qq.b));
        if (rr <= ll || bb <= tt) { rr = ll = tt = bb = 0; }
        return (Rect){ ll, tt, rr, bb };
    }

    const inline struct {
        const char32_t* codes;
        const char* beziers;
    } GLYPH_TABLE[] = {

        // Glyphs are sets of quadratic Bezier curves.  Each 2D Bezier needs
        // six points to describe it.  In order to keep things compact, we use
        // one character per point with the character '0' as the base.

        { U"\0", /* missing glyph */ "00@0@0@0@P@P@P0P0P000P0P" },

        { U"0", "80000<80@0@<0<0H8H@<@H8H>42D2D" },
        { U"1", "808H8H2280800H@H@H" },
        { U"2", "@H0H0H04208080@0@6@6@::<:<0@0H" },
        { U"3", "8H@H@A@A@:8:0A0H8H80?0?5?5?:8:801014" },
        { U"4", "<0<H<H<00>0>0>@>@>" },
        { U"5", "0D2H8H8H@H@@@@@88888280;0;000000@0@0" },
        { U"6", "88@8@@8H@H@@8H0H0@0@0888800008080@0@?4>080" },
        { U"7", "00@0@0@0686H" },
        { U"8", "@A@H8H8:@:@A8H0H0A8:0:0A8:?:?5?5?080801015151:8:" },
        { U"9", "80@0@7800007@7@>8>070>8>8H@H@@8H1H0B@@@8@8" },

        { U"\u215B", /* Fraction 1/8 */
            "@40D0D0:6:6:303:3:301111=H@H@E=H:H:E"
            ":E:B=B=B@B@E=B?B?@?@?>=>=>;>;@;@;B=B" },
        { U"\u2159", /* Fraction 1/6 */
            "@40D0D0:6:6:303:3:301111=H@H@E=H:H:E@E@"
            "B=B=B:B:E=>?>?" "?=>:>:B:B:E:E" },
        { U"\u2155", /* Fraction 1/5 */
            "@40D0D0:6:6:303:3:301111:>@>@>=H@H@E:F;"
            "H=H@E@A=A=A;A:B:>:B:B" },
        { U"\u00BC", /* Fraction 1/4 */
            "@40D0D0:6:6:303:3:301111?>:D:D:D@D@D?>?H?H" },
        { U"\u2153", /* Fraction 1/3 */
            "@40D0D0:6:6:303:3:301111:>@>@>@><B<B<B@B@E@E@H=H=H:H:E" },
        { U"\u215C", /* Fraction 3/8 */
            "@40D0D006060602424246467676:3:3:0:07=H@"
            "H@E=H:H:E:E:B=B=B@B@E=B?B?@?@?>=>=>;>;@;@;B=B" },
        { U"\u2156", /* Fraction 2/5 */
            "@40D0D0:6:6:30606363653636070:301002:>@"
            ">@>=H@H@E:F;H=H@E@A=A=A;A:B:>:B:B" },
        { U"\u00BD", /* Fraction 1/2 */
            "@40D0D0:6:6:303:3:301111:H@H@H=>@>@A@A@C=D=D:E:H=>;>:@" },
        { U"\u2157", /* Fraction 3/5 */
            "@40D0D006060602424246467676:3:3:0:07:>@"
            ">@>=H@H@E:F;H=H@E@A=A=A;A:B:>:B:B" },
        { U"\u215D", /* Fraction 5/8 */
            "@40D0D0060603:6:67081:3:676333331304000"
            "404=H@H@E=H:H:E:E:B=B=B@B@E=B?B?@?@?>=>=>;>;@;@;B=B" },
        { U"\u2154", /* Fraction 2/3 */
            "@40D0D0:6:6:30606363653636070:301002:>@"
            ">@>@><B<B<B@B@E@E@H=H=H:H:E" },
        { U"\u00BE", /* Fraction 3/4 */
            "@40D0D006060602424246467676:3:3:0:07?>:D:D:D@D@D?>?H?H" },
        { U"\u2158", /* Fraction 4/5 */
            "@40D0D500606066666505:5::>@>@>=H@H@E:F;H=H@E@A=A=A;A:B:>:B:B" },
        { U"\u215A", /* Fraction 5/6 */
            "@40D0D0060603:6:67081:3:676333331304000"
            "404=H@H@E=H:H:E@E@B=B=B:B:E=>?>?" "?=>:>:B:B:E:E" },
        { U"\u215E", /* Fraction 7/8 */
            "@40D0D00606060232:=H@H@E=H:H:E:E:B=B=B@"
            "B@E=B?B?@?@?>=>=>;>;@;@;B=B" },

        { U".", "8F9F9G8F7F7G7G7H8H8H9H9G" },
        { U",", "8F9F9G8F7F7G7G7H8H9G4N8H" },
        { U":", "8797988777787879898999988?9?9@8?7?7@7@7A8A9@9A8A" },
        { U";", "8797988777787879898999988?9?9@8?7?7@9@6D6D7@7A8A" },
        { U"=", "09@9@90?@?@?" },
        { U"&",
            "0@0F6F6F8F:D0@0>2<:D@>@>2<595959@D@D858659252659528285522225" },
        { U"|", "808H8H" },
        { U"+", "0<@<@<848D8D" },
        { U"-", "0<@<@<" },
        { U"~", "0<488<8<<@@<" },
        { U"*", "0<@<@<<44D4D44<D<D" },
        { U"%", ">42D2D:B:D<D:B:@<@<@>@>B>B>D<D262444446466666848262848" },
        { U"/", ">42D2D" },
        { U"\\", "24>D>D" },
        { U"^", "840<0<84@<@<" },
        { U"!", "8F9F9G8F7F7G7G7H8H8H9H9G8B8484" },
        { U"?", "82>2>8822228>8>::>:>8@8B8F9F9G8F7F7G7G7H8H8H9H9G" },
        { U"@",
            "86464<4<4B8B86<6<<<<<B8B82020<0<0F8F82@2@<<<<@>@>@@@@<8F<F=E" },
        { U"<", ">22<2<2<>F>F" },
        { U">", "22><><><2F2F" },
        { U"(", ":02<:H" },
        { U")", "60><6H" },
        { U"[", ":06060606H6H6H:H:H" },
        { U"]", "60:0:0:0:H:H:H6H6H" },
        { U"{", "<08084888<4<4<8<8@8D8H<H8@8D8D848888" },
        { U"}", "408084888<<<8@8<<<8@8D8D8D8H4H848888" },
        { U"#", "525F5F;2;F;F19?9?91?????" },
        { U"'", "828888" },
        { U"\"", "525858;2;8;8" },
        { U"`", "62:8:8" },
        { U"_", "0H@H@H" },
        { U"\u2032", /* Single Prime */ ":26868" },
        { U"\u301D", /* Reverse Double Prime Quotes */ "82<8<8428888" },
        { U"\u2033\u301E", /* Double Prime */ "824848<28888" },
        { U"\u2034", /* Triple Prime */ ":26868>2:8:8622828" },
        { U"\u00AB", /* Left Double Angle Quotes */
            "8<<8<88<<@<@4<8@8@4<8888" },
        { U"\u00BB", /* Right Double Angle Quotes */
            "488<8<8<4@4@88<<<<<<8@8@" },
        { U"\u2261", /* Triple Bar Definition */ "0<@<@<0A@A@A07@7@7" },
        { U"\u2245", /* Approximately Eq */ "0<@<@<0A@A@A87<;@7874307" },
        { U"\u2260", /* Not Equal */ "09@9@90?@?@?<44D4D" },
        { U"\u2248", /* Almost Equal */ "09458989<=@90?4;8?8?<C@?" },
        { U"\u2249", /* Not Almost Equal */ "89<=@98945098?<C@?8?4;0?<44D4D" },
        { U"\u2264", /* Less or Equal */ "0:@4@40:@@@@0D@D@D" },
        { U"\u2265", /* Greater or Equal */ "@:0@0@@:04040D@D@D" },
        { U"\u226A", /* Much Less */ "840<0<0<8D8D8<@4@48<@D@D" },
        { U"\u226B", /* Much Greater */ "84@<@<@<8D8D048<8<8<0D0D" },
        { U"\u00D7", /* Multiplication Sign */ "15?C?C1C?5?5" },
        { U"\u00F7", /* Division Sign */
            "0<@<@<8696978676777778888898978@9@9A8@7@7A7A7B8B8B9B9A" },
        { U"$", "808H8H9D>D>@2@2D8D84>4>8842428282:8<8<>>>@" },
        { U"\u20AC", /* Euro Currency */
            "9F3F3<3<329292=2>49F=F>D1:<:<:1>;>;>" },
        { U"\u00A3", /* Pound Currency */
            "2F>F>F2F6F6B666B6B6662:2:2>2>62<:<:<" },
        { U"\u00A5", /* Yen Currency */ "8:>2>28:22228:8F8F4:<:<:4><><>" },

        // Many of the Latin glyphs are used for Greek glyphs too
        { U"A\u0391", "800H0H80@H@H4><><>" },
        { U"B\u0392", "000H0H80>0>5>5>:8:8:@:@A@A@H8H0H8H8H0:8:8:008080" },
        { U"C",       "@6@0808000080@0H8H8H@H@B080@0@" },
        { U"D",       "80@0@88H@H@@000H0H0H8H8H008080@8@@@A" },
        { U"E\u0395", "000H0H0H@H@H00@0@00:<:<:" },
        { U"F",       "000H0H00@0@00:>:>:" },
        { U"G",       "@8@0808000080@0H8H080@0@8H@H@@8>@>@>@>@H@H" },
        { U"H\u0397", "000H0H@0@H@H0:@:@:" },
        { U"I\u0399", "808H8H00@0@00H@H@H" },
        { U"J",       "0@0H8H8H@H@@@0@@@@:0@0@0" },
        { U"K\u039A", "000H0H0@@0@06:@H@H" },
        { U"L",       "000H0H0H@H@H" },
        { U"M\u039C", "000H0H008B8B8B@0@0@0@H@H" },
        { U"N\u039D", "000H0H00@H@H@0@H@H" },
        { U"O\u039F", "80000880@0@80@0H8H8H@H@@080@0A@8@@@@" },
        { U"P\u03A1", "000H0H:00000:0@0@6@6@<:<:<0<0<" },
        { U"Q",       "80000880@0@80@0H8H8H@H@@080@0@@8@@@@8@@H@H" },
        { U"R",       "000H0H:00000:0@0@6@6@<:<:<0<0<:<@H@H" },
        { U"S",       "8<@<@B@B@H8H0B2H8H8<1<1616108080?0?4" },
        { U"T\u03A4", "808H8H00@0@0" },
        { U"U",       "0@0H8H8H@H@@0@0000@@@0@0" },
        { U"V",       "8H@0@08H0000" },
        { U"W",       "882H2H88>H>H>H@0@02H0000" },
        { U"X\u03A7", "00@H@H@00H0H" },
        { U"Y\u03A5", "8:@0@08:00008:8H8H" },
        { U"Z\u0396", "00@0@0@00H0H0H@H@H" },

        { U"\U0001D538", /* Double Struck A */ "800H0H80@H@H4><><><H6666" },
        { U"\U0001D539", /* Double Struck B */
            "000H0H80>0>5>5>:8:8:@:@A@A@H8H0H8H8H0:8:8:008080404H4H" },
        { U"\u2102",     /* Double Struck C */
            "@6@0808000080@0H8H8H@H@B080@0@414G4G" },
        { U"\U0001D53B", /* Double Struck D */
            "80@0@88H@H@@000H0H0H8H8H008080@8@@@A404H4H" },
        { U"\U0001D53C", /* Double Struck E */
            "000H0H0H@H@H00@0@00:<:<:404H4H" },
        { U"\U0001D53D", /* Double Struck F */ "000H0H00@0@00:>:>:404H4H" },
        { U"\U0001D53E", /* Double Struck G */
            "@8@0808000080@0H8H080@0@8H@H@@8>@>@>@>@H@H414G4G" },
        { U"\u210D",     /* Double Struck H */ "000H0H@0@H@H0:@:@:404H4H" },
        { U"\U0001D540", /* Double Struck I */ "00@0@00H@H@H606H6H:0:H:H" },
        { U"\U0001D541", /* Double Struck J */
            "0@0H8H8H@H@@@0@@@@80@0@0<0<G<G" },
        { U"\U0001D542", /* Double Struck K */ "000H0H404H4H@04<4<6:@H@H" },
        { U"\U0001D543", /* Double Struck L */ "000H0H0H@H@H404H4H" },
        { U"\U0001D544", /* Double Struck M */
            "000H0H008B8B8B@0@0@0@H@H494H4H<:<H<H" },
        { U"\u2115",     /* Double Struck N */ "000H0H00@H@H@0@H@H464H4H" },
        { U"\U0001D546", /* Double Struck O */
            "80000880@0@80@0H8H8H@H@@080@0A@8@@@@414G4G<1<G<G" },
        { U"\u2119",     /* Double Struck P */
         "000H0H:00000:0@0@6@6@<:<:<0<0<404H4H" },
        { U"\u211A",     /* Double Struck Q */
            "80000880@0@80@0H8H8H@H@@080@0@@8@@@@8@@H@H414G4G" },
        { U"\u211D",     /* Double Struck R */
            "000H0H:00000:0@0@6@6@<:<:<0<0<:<@H@H404H4H" },
        { U"\U0001D54A", /* Double Struck S */
            "8<@<@B@B@H8H0B2H8H8<1<1616108080?0?4414;4;<=<G<G" },
        { U"\U0001D54B", /* Double Struck T */ "00@0@0606H6H:0:H:H" },
        { U"\U0001D54C", /* Double Struck U */
            "0@0H8H8H@H@@0@0000@@@0@0404G4G" },
        { U"\U0001D54D", /* Double Struck V */ "8H@0@08H000040:B:B" },
        { U"\U0001D54E", /* Double Struck W */
            "880H0H88@H@H000H0H@0@G@G404@4@" },
        { U"\U0001D54F", /* Double Struck X */ "00<H<H40@H@H@00H0H" },
        { U"\U0001D550", /* Double Struck Y */
            "40:<:<006<6<6<6H6H:<:H:H:<@0@0" },
        { U"\u2124",     /* Double Struck Z */ "00@0@0@04H4H0H<0<00H@H@H" },

        { U"a", "0C0H8H0C0>8>8>@>@C@C@H8H@<@88888080<@<@H@H" },
        { U"b", "88080@0@0H8H88@8@@8H@H@@000H0H" },
        { U"c", "88080@0@0H8H88?8@=@C?H8H" },
        { U"d", "88080@0@0H8H88@8@@8H@H@@@0@H@H" },
        { U"e", "0@@@@@@@@88888080@0@0H8H8H>H@E" },
        { U"f", "08@8@86560;0;0?0@2656H6H4H8H8H" },
        { U"g", "88080@88@8@@0@0H8H@@@H8H@H@P8P8P1P0J@8@H@H" },
        { U"h", "88@8@@88080@000H0H@@@H@H" },
        { U"i", "81717281919292938372738358888;2858588;8H8H0H@H@H" },
        { U"j", "<8<J<J<J<P6P6P1P0L<84848<1=1=2<1;1;2;2;3<3<3=3=2" },
        { U"k", "000H0H0>@8@80>@H@H" },
        { U"l", "808H8H0H@H@H208080" },
        { U"m", "0>084848888<8<88<8<8@8@>070H0H@>@H@H8<8H8H" },
        { U"n", "88@8@@88180<080H0H@@@H@H" },
        { U"o", "88080@0@0H8H88@8@@8H@H@@" },
        { U"p", "88080@0@0H8H88@8@@8H@H@@080P0P" },
        { U"q", "88080@0@0H8H88@8@@8H@H@@@8@P@P" },
        { U"r", "282H2H98282>98>8?;" },
        { U"s", "88@8@<88080<0D0H8H8H@H@D0<0>8@8@@B@D" },
        { U"t", "646D6D6D6H:H:H>H>H18?8?8" },
        { U"u", "0@0H8H8H@H@B0@0808@8@H@H" },
        { U"v", "088H8H8H@8@8" },
        { U"w", "083H3H883H3H88=H=H@8=H=H" },
        { U"x", "08@H@H@80H0H" },
        { U"y", "@84P4P088H8H" },
        { U"z", "08@8@8@80H0H0H@H@H" },

        { U"\u03B1", /* Greek Lower Alpha */
            "<@<H@H<@<86868080@0@0H6H<@<H6H@8<8<@" },
        { U"\u03B2", /* Greek Lower Beta */
            "70202670<0<6<6<;6;6;><>A>A>H6H262P2P4;6;6;6H4H2G" },
        { U"\u03B3", /* Greek Lower Gamma */
            ">86H6H6H4L4O4O4P5P5P6P6O6O6H6H6H6B28" },
        { U"\u03B4", /* Greek Lower Delta */
            ">B>H8H8H2H2B>B>>8:2B2>8:8:373333307070;0=2" },
        { U"\u03B5", /* Greek Lower Epsilon */
            "88=8>:>F=H8H88282<2D2H8H=@2@2D=@2@2<" },
        { U"\u03F5", /* Greek Lunate Epsilon */
            ":8282@2@2H:H:H>H>H:8>8>82@<@<@" },
        { U"\u03B6", /* Greek Lower Zeta */ "20>0>0>02>2D2D2F8H8H:I:L:L:O8P" },
        { U"\u03B7", /* Greek Lower Eta */ "88>8>>>>>P>P88282>08282>2>2H2H" },
        { U"\u03B8", /* Greek Lower Theta */
            "80>0><80202<2<2H8H8H>H><2<><><" },
        { U"\u03B9", /* Greek Lower Iota */ "7H7I8I8I9I9H788888887D7H" },
        { U"\u03BA", /* Greek Lower Kappa */ "282H2H>8:82@2@>D>H" },
        { U"\u03BB", /* Greek Lower Lambda */ "882H2H88:D>H887240" },
        { U"\u03BC\u00B5", /* Greek Lower Mu */
            "2B2H8H8H>H>C282N2N>8>B>B>B>H@H" },
        { U"\u03BD", /* Greek Lower Nu */ "286@6H6H>@>8" },
        { U"\u03BE", /* Greek Lower Xi */
            ">060646468>8>8282@:H2F2@:H>J>L>L>P;P20>0>0" },
        { U"\u03BF", /* Greek Lower Omicron */ "88282@2@2H8H8H>H>@>@>888" },
        { U"\u03C0", /* Greek Lower Pi */ "48@8@848080:585H5H;8;D;D;D;H=H" },
        { U"\u03C1", /* Greek Lower Rho */ "88>8>@>@>H8H88282@2@2H8H2@2P2P" },
        { U"\u03C2", /* Greek Final Sigma */
            "88282@2@2H8H8H<H<L<L<N;N88=8>:" },
        { U"\u03C3", /* Greek Lower Sigma */
            "88282@2@2H8H8H>H>@88>8>@88@8@8" },
        { U"\u03C4", /* Greek Lower Tau */ "48@8@848080:8D8H<H888D8D" },
        { U"\u03C5", /* Greek Lower Upsilon */
            "2B2H8H8H>H>B282B2B>>>B>B>>>:<8" },
        { U"\u03C6", /* Greek Lower Phi */
            "2B2H8H>B>H8H;8888<;8>8><8<8P8P><>B>B2<2B2B2<2:48" },
        { U"\u03C7", /* Greek Lower Chi */ ">82P2P6<:L:L28586<:L;P>P" },
        { U"\u03C8", /* Greek Lower Psi */ "2B2H8H>B>H8H282B2B>8>B>B848P8P" },
        { U"\u03C9", /* Greek Lower Omega */
            "2@2H5H8@8H5H8@8H;H;H>H>@58282@;8>8>@" },

        { U"\u0393", /* Greek Cap Gamma */ "000H0H00@0@0@0@4@4" },
        { U"\u0394", /* Greek Cap Delta */ "800H0H80@H@H0H@H@H" },
        { U"\u0398", /* Greek Cap Theta */
            "80@0@88000080@0H8H8H@H@@080@0@@8@@@@2<><><" },
        { U"\u039B", /* Greek Cap Lambda */ "800H0H80@H@H" },
        { U"\u039E", /* Greek Cap Xi */
            "00@0@00H@H@H@0@2@20002020F0H0H@F@H@H3<=<=<" },
        { U"\u03A0", /* Greek Cap Pi */ "000H0H00@0@0@0@H@H" },
        { U"\u03A3", /* Greek Cap Sigma */ "@000008<00008<0H0H0H@H@H" },
        { U"\u03A6", /* Greek Cap Phi */
            "84@4@<84040<0<0D8D8D@D@<8H808020>0>02H>H>H" },
        { U"\u03A8", /* Greek Cap Psi */ "000808808H8H@0@8@8080@8@@8@@8@" },
        { U"\u03A9", /* Greek Cap Omega */
            "80@0@8800008:H@H@H6H0H0H6H0F0>:H@F@>@>@8@8080>0>" },
        { U"\u05D0", /* Hebrew Aleph */ "20@F@F02>H>H:<@<@26<0<0H" },

        { U"\u220F", /* Product */ "00@0@0404H4H<0<H<H" },
        { U"\u2211", /* Summation */ "00@0@08<00008<0H0H0H@H@H@H@D@D@0@4@4" },
        { U"\u2202", /* Partial */
            "2@2H8H>@>H8H88282@88>8>@>8>080>8>@>@804022" },
        { U"\u2207", /* Nabla */ "00@0@0@08H8H008H8H" },
        { U"\u2200", /* For All */ "008H8H@08H8H4<<<<<" },
        { U"\u2203", /* There Exists */ "00@0@0@0@H@H0H@H@H4<@<@<" },

        { U"\u222B", /* single integral */ "848D8D8480:08D8H6H" },
        { U"\u222C", /* double integral */
            "646D6D:4:D:D646080:4:0<06D6H4H:D:H8H" },
        { U"\u222D", /* triple integral */
            "848D8D8480:08D8H6H<4<D<D444D4D4D4H2H444060<4<0>0<D<H:H" },
        { U"\u2A0C", /* quadruple integral */
            ":4:D:D646D6D242D2D>4>D>D242040646080:4:0<0>4>0@02D"
            "2H0H6D6H4H:D:H8H>D>H<H" },
        { U"\u2A16", /* square integral */
            "5?;?;?;9;?;?59;9;9595?5?848D8D8480:08D8H6H" },
        { U"\u2A15", /* integral around point */
            "88<8<<88484<4<4@8@8@<@<<8288888280:08@8F8F8F8H6H8;"
            "9;9<8;7;7<7<7=8=9<9=8=" },
        { U"\u222E", /* contour integral */
            "848D8D8D8H6H8480:04<488888<8<<4<4@8@8@<@<<" },
        { U"\u222F", /* surface integral */
            "646D6D:4:D:D646080:4:0<06D6H4H:C:H8H2<28882<2@8@88"
            ">8><><>@8@" },
        { U"\u2230", /* volume integral */
            "8D8484444D4D<4<D<D4440608480:0<4<0>04D4H2H8D8H6H<C"
            "<H:H0<088888@8@<0<0@8@8@@@@<" },

        { U"\u2190", /* left arrow */ "0<@<@<0<5A5A0<5757" },
        { U"\u2191", /* up arrow */ "848D8D84393984=9=9" },
        { U"\u2192", /* right arrow */ "0<@<@<@<;A;A@<;7;7" },
        { U"\u2193", /* down arrow */ "848D8D8D=?=?8D3?3?" },
        { U"\u2194", /* left right arrow */
            "0<@<@<@<;A;A@<;7;70<57570<5A5A" },
        { U"\u2195", /* up down arrow */ "848D8D84393984=9=98D=?=?8D3?3?" },
        { U"\u2196", /* up left arrow */ "26>B>B262=2=269696" },
        { U"\u2197", /* up right arrow */ ">62B2B>6>=>=>67676" },
        { U"\u2198", /* down right arrow */ "26>B>B>B>;>;>B7B7B" },
        { U"\u2199", /* down left arrow */ ">62B2B2B9B9B2B2;2;" },
        { U"\u2921", /* up left down right arrow */
            "26>B>B269696262=2=>B7B7B>B>;>;" },
        { U"\u2922", /* up right down left arrow */
            ">62B2B>6>=>=>676762B9B9B2;2B2B" },
        { U"\u21BA", /* Clockwise Arrow */
            "8D@D@<8D0D0<>6@8@<26080<:3@3@3:3:9:9>6<4:3" },
        { U"\u21BB", /* Counter Clockwise Arrow */
            "8D@D@<8D0D0<>6@8@<26080<630303634426636969" },
        { U"\u21B6", /* CCW semicircle arrow  */
            ":6@6@<:6464<4>8:8:4>0:0:4<4>4>" },
        { U"\u21B7", /* CW semicircle arrow */
            "66060<66<6<<<>@:@:<>8:8:<<<><>" },
        { U"\u21D0", /* Double Left Arrow */ "0<57570<5A5A2:@:@:2>@>@>" },
        { U"\u21D1", /* Double Up Arrow */ "84393984=9=9666D6D:6:D:D" },
        { U"\u21D2", /* Double Right Arrow */ "@<;A;A@<;7;7>>0>0>>:0:0:" },
        { U"\u21D3", /* Double Down Arrow */ "8D=?=?8D3?3?6B6464:B:4:4" },
        { U"\u21D4", /* Double Left Right Arrow */
            "0<5A5A0<5757@<;A;A@<;7;72:>:>:2>>>>>" },
        { U"\u21D5", /* Double Up Down Arrow */
            "84393984=9=98D=?=?8D3?3?666B6B:6:B:B" },
        { U"\u21D6", /* Double Up Left Arrow */ "269696262=2=56>?>?29;B;B" },
        { U"\u21D7", /* Double Up Right Arrow */ ">67676>6>=>=;62?2?>95B5B" },
        { U"\u21D8", /* Double Down Right Arrow */
            ">B>;>;>B7B7B;B2929>?5656" },
        { U"\u21D9", /* Double Down Left Arrow */ "2B9B9B2B2;2;5B>9>92?;6;6" },

        { U"\u2208", /* Element Of */ "84040<0<0D8D84@4@40<@<@<8D@D@D" },
        { U"\u2209", /* Not Element Of */
            "0<0D8D0<048484@4@40<@<@<8D@D@D>02H2H" },
        { U"\u220B", /* Contains Member */ "84@4@<@<@D8D0484840<@<@<0D8D8D" },
        { U"\u220C", /* Not Contains */
            "@<@D8D@<@4840484840<@<@<0D8D8D>02H2H" },
        { U"\u22C2", /* Intersection */ "84040<84@4@<0<0D0D@<@D@D" },
        { U"\u22C3", /* Union */ "8D@D@<0<0D8D@4@<@<040<0<" },
        { U"\u2282", /* Is Subset */ "84040<0<0D8D8D@D@D84@4@4" },
        { U"\u2283", /* Is Superset */ "84@4@<@<@D8D8D0D0D048484" },
        { U"\u2284", /* Not Subset */ "84040<0<0D8D8D@D@D84@4@4>02H2H" },
        { U"\u2285", /* Not Superset */ "84@4@<@<@D8D8D0D0D048484>02H2H" },
        { U"\u2286", /* Is Subset or Eq */ "64040:0:0@6@6@@@@@64@4@40D@D@D" },
        { U"\u2287", /* Is Superset or Eq */
            ":4@4@:@:@@:@04:4:40@:@:@0D@D@D" },
        { U"\u2288", /* Not Subset or Eq */
            "64040:0:0@6@6@@@@@64@4@40D@D@D>02H2H" },
        { U"\u2289", /* Not Supset or Eq */
            ":4@4@:@:@@:@04:4:40@:@:@0D@D@D>02H2H" },
        { U"\u228A", /* Strict Subset */
            "64040:0:0@6@64@4@46@@@@@0D@D@D:B6F6F" },
        { U"\u228B", /* Strict Supset */
            ":4@4@:@:@@:@:@0@0@04:4:40D@D@D:B6F6F" },

        { U"\u228F", /* Square Subset */ "0D@D@D04@4@4040D0D" },
        { U"\u2290", /* Square Superset */ "0D@D@D04@4@4@4@D@D" },
        { U"\u2291", /* Square Subset or Eq */ "0D@D@D@@0@0@04@4@4040@0@" },
        { U"\u2292", /* Square Superset or Eq */ "0D@D@D@@0@0@04@4@4@4@@@@" },
        { U"\u22E2", /* Not Square Subset or Eq */
            "0D@D@D0@@@@@04@4@4>02H2H040@0@" },
        { U"\u22E3", /* Not Square Supset or Eq */
            "0D@D@D0@@@@@04@4@4>02H2H@4@@@@" },
        { U"\u2293", /* Square Intersection */ "04@4@4040D0D@4@D@D" },
        { U"\u2294", /* Square Union */ "0D@D@D040D0D@4@D@D" },
        { U"\u22C0", /* Conjunction */ "84@D@D840D0D" },
        { U"\u22C1", /* Disjunction */ "048D8D8D@4@4" },

        { U"\u00AC", /* Not Sign */ "0<@<@<@<@B@B" },
        { U"\u2205", /* Empty Set */ "84@4@<84040;0<0D8D8D@D@<@40D0D" },
        { U"\u00B1", /* Plus Over Minus */ "08@8@8808@8@0D@D@D" },
        { U"\u2213", /* Minus Over Plus */ "888H8H0@@@@@04@4@4" },
        { U"\u22A2", /* Left Tee */ "040D0D0<@<@<" },
        { U"\u22A3", /* Right Tee */ "@4@D@D0<@<@<" },
        { U"\u22A4", /* Top Tee */ "04@4@4848D8D" },
        { U"\u22A5", /* Bottom Tee */ "0D@D@D8D8484" },

        { U"\u2295", /* Circle Add */ "84@4@<84040<0<0D8D8D@D@<3<=<=<878A8A" },
        { U"\u2296", /* Circle Sub */ "84@4@<84040<0<0D8D8D@D@<3<=<=<" },
        { U"\u2297", /* Circle Mul */ "84@4@<84040<0<0D8D8D@D@<48<@<@<84@4@" },
        { U"\u2298", /* Circle Div */ "84@4@<84040<0<0D8D8D@D@<<84@4@" },
        { U"\u2299", /* Circle Dot */
            "84@4@<84040<0<0D8D8D@D@<8;9;9<8;7;7<7<7=8=8=9=9<" },
        { U"\u229A", /* Circle Compose */
            "84@4@<84040<0<0D8D8D@D@<89;9;<89595<5<5?8?8?;?;<" },
        { U"\u229B", /* Circle Asterisk */
            "84@4@<84040<0<0D8D8D@D@<3<=<=<;75A5A57;A;A" },
        { U"\u229C", /* Circle Equal */
            "84040<0<0D8D84@4@<8D@D@<3>=>=>3:=:=:" },

        { U"\u229E", /* Square Plus */
            "0D@D@D@4@D@D04@4@4040D0D3<=<=<878A8A" },
        { U"\u229F", /* Square Minus */ "0D@D@D@4@D@D04@4@4040D0D3<=<=<" },
        { U"\u22A0\u2612", /* Box with X */
            "04@4@4@4@D@D0D@D@D040D0D37=A=A=73A3A" },
        { U"\u22A1", /* Square Dot */
            "04@4@4040D0D0D@D@D@4@D@D8;9;9<8;7;7<7<7=8=8=9=9<" },
        { U"\u2610", /* Square Box */ "04@4@4@4@D@D0D@D@D040D0D" },
        { U"\u2611", /* Box with Check */
            "04@4@4@4@D@D0D@D@D040D0D7A6?3>7A8;=7" },

        { U"\u2308", /* Left Ceiling */ "<06060606H6H" },
        { U"\u2309", /* Right Ceiling */ "40:0:0:0:H:H" },
        { U"\u230A", /* Left Floor */ "606H6H6H<H<H" },
        { U"\u230B", /* Right Floor */ ":0:H:H4H:H:H" },

        { U"\u223F", /* Sine Wave */
            "789@9@7864449@:D<D<D>D?@?@@<@<0<1818442418" },
        { U"\u221A", /* Square Root */ "@0<0<0<04H4H2@4H4H0@2@2@" },
        { U"\u221B", /* Cube Root */
            "@0<0<0<04H4H2@4H4H0@2@2@686:4:6866464666646462424232234:3:29" },
        { U"\u221C", /* Fourth Root */
            "@0<0<0<04H4H2@4H4H0@2@2@626:6:622727277777" },

        { U"\u2234", /* Therefore */
            "?B@B@C?B>B>C>C>D?D?D@D@C1B2B2C2C2D1D0C0D1D0C0B1B84"
            "9495959686757686757484" },
        { U"\u2235", /* Because */
            "?4@4@5?6@6@5>5>6?6>5>4?41424251404050506161626258D"
            "9D9C8B9B9C7C7B8B7C7D8D" },
        { U"\u2236", /* Ratio */ "8494958474757576868696958D"
            "9D9C7C7D8D7C7B8B8B9B9C" },
        { U"\u2237", /* Proportional */
            "?4@4@5?4>4>5?6@6@5>5>6?61424251404050506161626250C"
            "0D1D1D2D2C1B2B2C1B0B0C?B@B@C?B>B>C>C>D?D?D@D@C" },
        { U"\u22EE", /* Vertical Ellipsis */
            "8;9;9<8;7;7<7<7=8=8=9=9<8494958474757576868696958B"
            "9B9C8B7B7C7C7D8D8D9D9C" },
        { U"\u22EF", /* Horizontal Ellipsis */
            "8;9;9<8;7;7<7<7=8=8=9=9<?;@;@<?;>;><><>=?=?=@=@<1;"
            "0;0<0<0=1=1=2=2<1;2;2<" },
        { U"\u22F0", /* Down Left Up Right Ellipsis */
            "8;9;9<8;7;7<7<7=8=8=9=9<=6>6>7=6<6<7<7<8=8=8>8>73@"
            "4@4A3@2@2A2A2B3B3B4B4A" },
        { U"\u22F1", /* Down Right Up Left Ellipsis */
            "8;9;9<8;7;7<7<7=8=8=9=9<=@>@>A=@<@<A<A<B=B=B>B>A36"
            "4647362627272838384847" },

        { U"\u00A9", /* Copyright */
            "84@4@<84040<0<0D8D8D@D@<87575<5<5A8A8A;A;>87;7;:" },
        { U"\u00AE", /* Registered Mark */
            "84@4@<84040<@<@D8D0<0D8D575A5A;:;787;:;=8=8=5=5=5787879=;A;A" },
        { U"\u2122", /* Trade Mark TM */
            "40484810707090989890<5<5<5?0?0?0?8?8" },

        { U"\u22C6", /* Five Point Star */ "8<86868<=:=:8<3:3:8<;A;A8<5A5A" },
        { U"\u2220", /* Angle */ "0D@D@D840D0D" },
        { U"\u221F", /* Perpendicular */ "040D0D0D@D@D0?5?5?5?5D5D" },
        { U"\u2225", /* Parallel */ "505H5H;0;H;H" },
        { U"\u2226", /* Not Parallel */ "505H5H;0;H;H@80@0@" },
        { U"\u00A7", /* Section */
            "88383<88=8=<3<3@8@8@=@=<88282424208080=0>28@>@>D>D>H8H2F3H8H" },
        { U"\u2020", /* Single Dagger */ "808@8@44<4<4" },
        { U"\u2021", /* Double Dagger */ "808@8@44<4<44<<<<<" },

        { U"\u2218", /* Compose Operator */ "88<8<<88484<4<4@8@8@<@<<" },
        { U"\u221D", /* Proportional */
            "0<08580<0@5@5888:<:<8@5@:<<8@8:<<@@@" },
        { U"\u221E", /* Infinity */
            "@<@@<@<8@8@<48080<0<0@4@48688<8<:@<@7>6@4@9::8<8" },
        { nullptr, nullptr } // sentinel
    };

    static inline void quadratic_roots(
        double xx[2], double aa, double bb, double cc
    ) {
        if (aa == 0) {
            if (bb == 0) {
                xx[0] = xx[1] = NAN;
                return;
            }
            xx[0] = -cc/bb;
            xx[1] = NAN;
            return;
        }
        double descriminant = bb*bb - 4.0*aa*cc;
        if (descriminant < 0.0) {
            xx[0] = xx[1] = NAN;
            return;
        }
        double radical = sqrt(descriminant);

        if (bb < 0.0) {
            xx[0] = (2.0*cc)/(-bb + radical);
            xx[1] = (-bb + radical)/(2.0*aa);
        } else {
            xx[0] = (-bb - radical)/(2.0*aa);
            xx[1] = (2.0*cc)/(-bb - radical);
        }
    }

    static inline void cubic_roots(
        double xx[3], double aa, double bb, double cc, double dd
    ) {
        if (isnan(aa) || isnan(bb) || isnan(cc) || isnan(dd)) {
            throw "internal error: NaN in cubic_roots";
        }
        if (isinf(aa) || isinf(bb) || isinf(cc) || isinf(dd)) {
            throw "internal error: Inf in cubic_roots";
        }

        // This algorithm closely follows the recipe on Wikipedia or Wolfram:
        //
        //    https://en.wikipedia.org/wiki/Cubic_equation
        //    https://mathworld.wolfram.com/CubicFormula.html
        //

        if (aa == 0.0) { // it's not a cubic
            xx[2] = NAN;
            quadratic_roots(xx, bb, cc, dd);
            return;
        }

        // monic polynomial
        double uu = bb/aa;
        double vv = cc/aa;
        double ww = dd/aa;

        // change of variable x = t - u/3
        double change = -uu/3.0;

        // negative descriminant
        double pp = vv/3.0 - uu*uu/9.0;
        double qq = uu*(uu*uu/27.0 - vv/6.0) + ww/2.0;
        double negdisc = pp*pp*pp + qq*qq;

        if (negdisc == 0.0) { // repeated roots
            if (pp == 0.0) { // triple root
                xx[0] = xx[1] = xx[2] = change;
                return;
            }
            double dup = -qq/pp;
            double uno = -dup*2.0;
            xx[0] = change + uno;
            xx[1] = change + dup;
            xx[2] = change + dup;
            return;
        }

        if (negdisc > 0) { // one root
            double root = sqrt(negdisc);
            xx[0] = cbrt(-qq + root) + cbrt(-qq - root) + change;
            xx[1] = xx[2] = NAN;
            return;
        }

        // three roots
        double fract = fmax(-1.0, fmin(qq/(pp*sqrt(-pp)), +1.0));
        double common = acos(fract)/3.0;
        double front = 2.0*sqrt(-pp);
        double angle = 2.0*M_PI/3.0;
        xx[0] = front*cos(common - 0.0*angle) + change;
        xx[1] = front*cos(common - 1.0*angle) + change;
        xx[2] = front*cos(common - 2.0*angle) + change;
    }

    static inline double quadratic_bezier(
        double u0, double u1, double u2, double tt
    ) {
        double aa = tt*tt;
        double bb = 2.0*tt*(1.0 - tt);
        double cc = (1.0 - tt)*(1.0 - tt);
        return aa*u0 + bb*u1 + cc*u2;
    }

    static inline double sqr(double xx) { return xx*xx; }

    static inline double bezier_distance(
        double x0, double y0, double x1, double y1,
        double x2, double y2, double xx, double yy
    ) {
        if (
            (x1 == x0 && y1 == y0) ||
            (x1 == x2 && y1 == y2)
        ) {
            // this quadratic is actually linear, so the
            // numerics are better if we use the midpoint
            x1 = 0.5*(x0 + x2);
            y1 = 0.5*(y0 + y2);
        };

        //
        // from sympy import *
        //
        // tt = symbols('tt')
        // x0, y0, x1, y1 = symbols('x0 y0 x1 y1')
        // x2, y2, xx, yy = symbols('x2 y2 xx yy')
        //
        // aa = tt*tt
        // bb = 2*tt*(1 - tt)
        // cc = (1 - tt)*(1 - tt)
        //
        // bez_x = aa*x0 + bb*x1 + cc*x2
        // bez_y = aa*y0 + bb*y1 + cc*y2
        //
        // dist_squared = (bez_x - xx)**2 + (bez_y - yy)**2
        // derivative = diff(dist_squared, tt)
        //
        // print(collect(expand(derivative), tt))
        //

        double aa = (
            x0*x0 + x2*x2 + y0*y0 + y2*y2
            + 2.0*x0*x2 + 2.0*y0*y2
            + 4.0*x1*x1 - 4.0*x0*x1 - 4.0*x1*x2
            + 4.0*y1*y1 - 4.0*y0*y1 - 4.0*y1*y2
        );
        double bb = 3.0*(
            x0*x1 - x2*x2 - x0*x2 + y0*y1 - y0*y2 - y2*y2
            - 2.0*x1*x1 - 2.0*y1*y1 + 3.0*x1*x2 + 3.0*y1*y2
        );
        double cc = (
            x0*x2 - x0*xx - x2*xx + y0*y2 - y0*yy - y2*yy
            + 2.0*x1*x1 + 2.0*x1*xx + 2.0*y1*y1 + 2.0*y1*yy
            + 3.0*x2*x2 + 3.0*y2*y2 - 6.0*x1*x2 - 6.0*y1*y2
        );
        double dd = (
            x1*x2 - x1*xx - x2*x2 + x2*xx +
            y1*y2 - y1*yy - y2*y2 + y2*yy
        );

        double head = sqr(x0 - xx) + sqr(y0 - yy);
        double tail = sqr(x2 - xx) + sqr(y2 - yy);
        double min = fmin(head, tail);

        double roots[3];
        cubic_roots(roots, aa, bb, cc, dd);
        for (long ii = 0; ii<3; ii++) {
            double tt = roots[ii];
            if (isnan(tt) || isinf(tt)) continue;
            if (0.0 < tt && tt < 1.0) {
                double uu = quadratic_bezier(x0, x1, x2, tt);
                double vv = quadratic_bezier(y0, y1, y2, tt);
                double dist = sqr(uu - xx) + sqr(vv - yy);
                min = fmin(min, dist);
            }
        }

        return min;
    }

    static inline Field make_field(const char* paths) {
        Field result;

        const double xscale = GLYPH_XRANGE*1.0/FIELD_XRANGE;
        const double yscale = GLYPH_YRANGE*1.0/FIELD_YRANGE;
        for (long ii = 0; ii<FIELD_HEIGHT; ++ii) {
            for (long jj = 0; jj<FIELD_WIDTH; ++jj) {
                double xx = (jj - FIELD_MARGIN)*xscale;
                double yy = (ii - FIELD_MARGIN)*yscale;
                double min = INFINITY;
                for (const char* pp = paths; *pp != 0; pp += 6) {
                    double x0 = pp[0] - '0', y0 = pp[1] - '0';
                    double x1 = pp[2] - '0', y1 = pp[3] - '0';
                    double x2 = pp[4] - '0', y2 = pp[5] - '0';

                    double dist2 = bezier_distance(
                        x0, y0, x1, y1, x2, y2, xx, yy
                    );
                    min = fmin(min, dist2);
                }
                result.data[ii][jj] = sqrt(min);
            }
        }

        return result;
    }

    static inline double interp_field(const Field& field, double ix, double iy) {
        double xx = ix + FIELD_MARGIN;
        double yy = iy + FIELD_MARGIN;
        if (xx < 0.0 || yy < 0.0) { return INFINITY; }
        if (xx > FIELD_WIDTH  - 1.0) { return INFINITY; }
        if (yy > FIELD_HEIGHT - 1.0) { return INFINITY; }

        long ii = long(yy);
        long jj = long(xx);
        double fi = yy - ii;
        double fj = xx - jj;

        double aa = field.data[ii + 0][jj + 0];
        double bb = field.data[ii + 0][jj + 1];
        double cc = field.data[ii + 1][jj + 0];
        double dd = field.data[ii + 1][jj + 1];
        return (
            aa*(1.0 - fi)*(1.0 - fj) +
            bb*(1.0 - fi)*(fj - 0.0) +
            cc*(fi - 0.0)*(1.0 - fj) +
            dd*(fi - 0.0)*(fj - 0.0)
        );
    }

    template<class dstpixel, class srcpixel>
    static inline void render_field(
        dstpixel* image, long image_w,
        Rect bounds, double px, double py,
        double matrix_aa, double matrix_bb,
        double matrix_cc, double matrix_dd,
        const Field& field, srcpixel color, double thresh, long stride=-1
    ) {
        if (stride < 0) {
          stride = image_w;
        }

        const static struct {
            double dx, dy;
        } sampling[16] = {
            // I'm not sure how to pick an "optimal" sampling pattern for
            // rendering text, but this seems to work ok.  It's a radial
            // pattern that avoids the corners a bit.  It has a mean of .5 for
            // both dx and dy.  The variance of dx and dy is equal, and the
            // covariance and skew is zero.  If considered on a 16x16 grid,
            // each sample is in a different row and column from every other
            // sample (which is good for horizontal and vertical edges).
            { 0.53125, 0.03125 }, { 0.28125, 0.09375 },
            { 0.78125, 0.15625 }, { 0.15625, 0.21875 },
            { 0.90625, 0.28125 }, { 0.40625, 0.34375 },
            { 0.65625, 0.40625 }, { 0.03125, 0.46875 },
            { 0.96875, 0.53125 }, { 0.34375, 0.59375 },
            { 0.59375, 0.65625 }, { 0.09375, 0.71875 },
            { 0.84375, 0.78125 }, { 0.21875, 0.84375 },
            { 0.71875, 0.90625 }, { 0.46875, 0.96875 },
        };

        for (long ii = bounds.t; ii<bounds.b; ++ii) {
            for (long jj = bounds.l; jj<bounds.r; ++jj) {
                double ox = jj - px;
                double oy = ii - py;
                long count = 0;
                for (long kk = 0; kk<16; kk++) {
                    double dx = sampling[kk].dx;
                    double dy = sampling[kk].dy;
                    double xx = ox + dx;
                    double yy = oy + dy;
                    double ix = matrix_aa*xx + matrix_bb*yy;
                    double iy = matrix_cc*xx + matrix_dd*yy;
                    if (interp_field(field, ix, iy) <= thresh) {
                        count += 1;
                    }
                }
                long index = ii*stride + jj;
                image[index] = blend(image[index], color, 16*count);
            }
        }
    }

    const Field& Font::lookup_field(char32_t code) {
        long index = 0;
        if (code_to_index.count(code)) {
            index = code_to_index[code];
        }
        if (!index_to_field.count(index)) {
            index_to_field[index] = make_field(
                GLYPH_TABLE[index].beziers
            );
        }
        return index_to_field[index];
    }

    struct Matrix { double a, b, c, d; };
    struct Vector { double x, y; };

    static inline Matrix operator*(const Matrix &u, const Matrix &v) {
        return Matrix {
            u.a*v.a + u.b*v.c, u.a*v.b + u.b*v.d,
            u.c*v.a + u.d*v.c, u.c*v.b + u.d*v.d
        };
    }

    static inline Vector operator*(const Matrix &u, const Vector &v) {
        return Vector { u.a*v.x + u.b*v.y, u.c*v.x + u.d*v.y };
    }

    static inline Matrix inverse(const Matrix& u) {
        double inv = 1.0/(u.a*u.d - u.b*u.c);
        return Matrix { u.d*inv, -u.b*inv, -u.c*inv, u.a*inv };
    }

    Font::Font() {
        for (long ii = 0; GLYPH_TABLE[ii].codes != nullptr; ++ii) {
            const char32_t* codes = GLYPH_TABLE[ii].codes;
            for (long jj = 0; codes[jj] != 0; ++jj) {
                if (code_to_index.count(codes[jj])) {
                    throw "internal error: code point is repeated";
                }
                code_to_index[codes[jj]] = ii;
            }
        }
    }

    template<class dstpixel, class srcpixel, class chartype>
    void Font::draw(
        dstpixel* pixmap, long width, long height, srcpixel color,
        double xx, double yy, const chartype* text, const Options& opts,
        long stride
    ) {
        if (opts.width == 0.0 || opts.height == 0.0) return;

        Rect clip = { 0, 0, width, height };
        clip = intersect(clip, opts.clip);
        if (clip.r <= clip.l || clip.b <= clip.t) return;

        double angle = -M_PI/180.0*opts.degrees;
        double cc = cos(angle), ss = sin(angle);
        Matrix rot = { cc, -ss, ss, cc };
        Matrix mat = {
            opts.width, -opts.oblique*opts.width, 0.0, opts.height
        };
        Matrix fwd = rot*mat;
        Matrix fld = { FIELD_XRANGE, 0, 0, FIELD_YRANGE };
        Matrix inv = fld*inverse(fwd);

        double margin_x = FIELD_MARGIN*1.0/FIELD_XRANGE;
        double margin_y = FIELD_MARGIN*1.0/FIELD_YRANGE;
        Vector tl = fwd*Vector{ 0.0 - margin_x, 0.0 - margin_y };
        Vector tr = fwd*Vector{ 1.0 + margin_x, 0.0 - margin_y };
        Vector bl = fwd*Vector{ 0.0 - margin_x, 1.0 + margin_y };
        Vector br = fwd*Vector{ 1.0 + margin_x, 1.0 + margin_y };

        double bound_l = std::min({ tl.x, tr.x, bl.x, br.x }) - 1.0;
        double bound_t = std::min({ tl.y, tr.y, bl.y, br.y }) - 1.0;
        double bound_r = std::max({ tl.x, tr.x, bl.x, br.x }) + 1.0;
        double bound_b = std::max({ tl.y, tr.y, bl.y, br.y }) + 1.0;

        double col_dx =  opts.width*(1.0 + opts.spacing)*cc;
        double col_dy =  opts.width*(1.0 + opts.spacing)*ss;
        double row_dx = -opts.height*(1.0 + opts.leading)*ss;
        double row_dy =  opts.height*(1.0 + opts.leading)*cc;

        double thresh = fmin(fmax(opts.weight, 0.5), 4.0);

        long row = 0, col = 0;
        for (long ii = 0; text[ii] != 0; ++ii) {
            char32_t cc = (char32_t)text[ii];
            if (cc == ' ') { col += 1; continue; }
            if (cc == '\n') { col = 0; row += 1; continue; }
            if (cc == '\t') { col += 4 - col%4; continue; }

            double delta_x = row*row_dx + col*col_dx;
            double delta_y = row*row_dy + col*col_dy;
            double glyph_x = xx + delta_x;
            double glyph_y = yy + delta_y;
            long glyph_l = lrint(bound_l + glyph_x);
            long glyph_t = lrint(bound_t + glyph_y);
            long glyph_r = lrint(bound_r + glyph_x);
            long glyph_b = lrint(bound_b + glyph_y);
            Rect bounds = { glyph_l, glyph_t, glyph_r, glyph_b };
            bounds = intersect(clip, bounds);

            const Field& field = lookup_field(cc);
            render_field(
                pixmap, width, bounds, glyph_x, glyph_y,
                inv.a, inv.b, inv.c, inv.d,
                field, color, thresh, stride
            );
            col += 1;
        }
    }
}
