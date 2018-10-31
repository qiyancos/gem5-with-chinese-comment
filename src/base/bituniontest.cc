/*
 * Copyright 2014 Google, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Gabe Black
 */

#include <gtest/gtest.h>

#include <cassert>
#include <iostream>
#include <type_traits>

#include "base/bitunion.hh"
#include "base/cprintf.hh"

using namespace std;

namespace {

BitUnion64(SixtyFour)
    Bitfield<39, 32> byte5;
	// Bitfield对应的是BitfieldType<Unsigned<Storage, first, last>>
	// 很明显这里没有定义Storage类型对应的实际类型
    Bitfield<2> bit2;
    BitfieldRO<39, 32> byte5RO;
    BitfieldWO<39, 32> byte5WO;
    SubBitUnion(byte6, 47, 40)
        Bitfield<43, 42> bits43To42;
        Bitfield<41> bit41;
        SignedBitfield<41> bit41Signed;
    EndSubBitUnion(byte6)
    SignedBitfield<47, 40> byte6Signed;
    SignedBitfieldRO<47, 40> byte6SignedRO;
    SignedBitfieldWO<47, 40> byte6SignedWO;
EndBitUnion(SixtyFour)

// 下面的代码是将上面的代码的宏替换成实际代码后实际生成的代码
/*
class BitfieldUnderlyingClassesSixtyFour : 
	public BitfieldBackend::BitfieldTypes<uint64_t> 
{ 
	protected: 
        typedef uint64_t __StorageType; 
        friend BitfieldBackend::BitUnionBaseType<BitfieldBackend::\
				BitUnionOperators<BitfieldUnderlyingClassesSixtyFour>>; 
        friend BitfieldBackend::BitUnionBaseType< \
                BitfieldUnderlyingClassesSixtyFour>; 
		// 目前还没发现引入这两个结构体有啥用？？？
	public: 
        union { 
			// union就是共用体。里面定义的内容共享一个起始地址
			// union的地址空间等于里面定义内容中占用空间最大对应的空间
			
            uint64_t __storage;
			// 这个是本类中唯一的数据实体
			Bitfield<39, 32> byte5;
			Bitfield<2> bit2;
			BitfieldRO<39, 32> byte5RO;
			BitfieldWO<39, 32> byte5WO;
			class
			{ 
				public: 
					union {
						Bitfield<47, 40> __storage;
						Bitfield<43, 42> bits43To42;
						Bitfield<41> bit41;
						SignedBitfield<41> bit41Signed;
					};
				
				inline operator __StorageType () const { return __storage; }
				// 对于类型转换类型的运算符重载，并不需要说明返回数据类型
				// 因为返回数据类型就是强制转换运算符中名称对应的类型
				// 注意这里不仅可以作为显示转换处理，也可以作为隐式转换时的调用对象
				// 参考网址：https://blog.csdn.net/mlyjqx/article/details/73799844
				inline __StorageType operator = (const __StorageType & _storage) { 
					return __storage = _storage;
				}
				
			} byte6;
			SignedBitfield<47, 40> byte6Signed;
			SignedBitfieldRO<47, 40> byte6SignedRO;
			SignedBitfieldWO<47, 40> byte6SignedWO;
		}; 
};
typedef BitfieldBackend::BitUnionOperators<BitfieldUnderlyingClassesSixtyFour> SixtyFour;
// 这里的定义使得BitUnionOperators中的重载运算符可以用于
// BitfieldUnderlyingClassesSixtyFour这一个类的计算
*/

BitUnion64(EmptySixtyFour)
EndBitUnion(EmptySixtyFour)

BitUnion32(EmptyThirtyTwo)
EndBitUnion(EmptyThirtyTwo)

BitUnion16(EmptySixteen)
EndBitUnion(EmptySixteen)

BitUnion8(EmptyEight)
EndBitUnion(EmptyEight)

class SplitField
{
  protected:
    BitUnion64(In)
        Bitfield<15, 12> high;
        Bitfield<7, 4> low;
    EndBitUnion(In)

    BitUnion64(Out)
        Bitfield<7, 4> high;
        Bitfield<3, 0> low;
    EndBitUnion(Out)
  public:
    uint64_t
    getter(const uint64_t &storage) const
    {
        Out out = 0;
        In in = storage;
        out.high = in.high;
		// 我们从左向右看来理解这个计算的处理过程
		// 首先会对左边调用重载运算符 Type operator=(BitfieldType<Base> const & other)
		// 然后会调用BitfieldTypeImpl的重载运算符“=”返回*this = (Type)other
		// 这时候就会对右边的in.high进行显示的类型转换
		// 而左边则会继续调用运算符Type operator=(const Type val)
		// 从而调用in.high的getter函数获取到需要的数据
		// 调用out.high的setter函数将右边得到的数据进行赋值操作
        out.low = in.low;
        return out;
		// 返回的时候需要进行强制类型转换
		// 这时候上面BitUnionOprators对强制类型转换定义就有作用了
    }

    void
    setter(uint64_t &storage, uint64_t val)
    {
        Out out = val;
        In in = 0;
        in.high = out.high;
        in.low = out.low;
        storage = in;
    }
};

BitUnion64(Split)
    BitfieldType<SplitField> split;
EndBitUnion(Split)

struct ContainingStruct
{
    BitUnion64(Contained)
        Bitfield<63, 60> topNibble;
    EndBitUnion(Contained)

    Contained contained;
};
// 只定义了60~63位域的处理类对应的结构体

uint64_t
containingFunc(uint64_t init_val, uint64_t fieldVal)
{
    BitUnion32(Contained)
        Bitfield<16, 15> field;
    EndBitUnion(Contained)

    Contained contained = init_val;
    contained.field = fieldVal;
    return contained;
}
// 包含类声明和构造的函数
// init_val对应于64位__storage的初始化数据
// 而field_val的低2位被覆盖到__storage的15~16位

} // anonymous namespace

// Declare these as global so g++ doesn't ignore them. Initialize them in
// various ways.
EmptySixtyFour emptySixtyFour = 0;
EmptyThirtyTwo emptyThirtyTwo;
EmptySixteen emptySixteen;
EmptyEight emptyEight(0);

class BitUnionData : public testing::Test {
  protected:
    SixtyFour sixtyFour;
    Split split;

    void SetUp() override { sixtyFour = 0; split = 0; }
	// 初始化两个数据的实体变量为0

    template <typename T>
    uint64_t templatedFunction(T) { return 0; }

    template <typename T>
    uint64_t
    templatedFunction(BitUnionType<T> u)
    {
        BitUnionBaseType<T> b = u;
        return b;
    }
};
// 该类定义了两个不同的数据处理类型sictyFour和split

TEST_F(BitUnionData, NormalBitfield)
{
    EXPECT_EQ(sixtyFour.byte5, 0);
    sixtyFour.byte5 = 0xff;
    EXPECT_EQ(sixtyFour, 0xff00000000);
    sixtyFour.byte5 = 0xfff;
    EXPECT_EQ(sixtyFour, 0xff00000000);
    EXPECT_EQ(sixtyFour.byte5, 0xff);
}

TEST_F(BitUnionData, SingleBitfield)
{
    EXPECT_EQ(sixtyFour.bit2, 0);
    sixtyFour.bit2 = 0x1;
    EXPECT_EQ(sixtyFour, 0x4);
    EXPECT_EQ(sixtyFour.bit2, 0x1);
}

TEST_F(BitUnionData, ReadOnlyBitfield)
{
    EXPECT_EQ(sixtyFour.byte5RO, 0);
    sixtyFour.byte5 = 0xff;
    EXPECT_EQ(sixtyFour.byte5RO, 0xff);
}

TEST_F(BitUnionData, WriteOnlyBitfield)
{
    sixtyFour.byte5WO = 0xff;
    EXPECT_EQ(sixtyFour, 0xff00000000);
}

TEST_F(BitUnionData, SubBitUnions)
{
    EXPECT_EQ(sixtyFour.byte6.bit41, 0);
    sixtyFour.byte6 = 0x2;
    EXPECT_EQ(sixtyFour.byte6.bit41, 1);
    sixtyFour.byte6.bits43To42 = 0x3;
    EXPECT_EQ(sixtyFour.byte6, 0xe);
    sixtyFour.byte6 = 0xff;
    sixtyFour.byte6.bit41 = 0;
    EXPECT_EQ(sixtyFour, 0xfd0000000000);
}

TEST_F(BitUnionData, SignedBitfields)
{
    sixtyFour.byte6 = 0xff;
    EXPECT_EQ(sixtyFour.byte6Signed, -1);
    EXPECT_EQ(sixtyFour.byte6SignedRO, -1);
    sixtyFour.byte6SignedWO = 0;
    EXPECT_EQ(sixtyFour.byte6Signed, 0);
    EXPECT_EQ(sixtyFour.byte6SignedRO, 0);
    EXPECT_EQ(sixtyFour.byte6, 0);
}

TEST_F(BitUnionData, InsideStruct)
{
    ContainingStruct containing;
    containing.contained = 0;
    containing.contained.topNibble = 0xd;
    EXPECT_EQ(containing.contained, 0xd000000000000000);
}

TEST_F(BitUnionData, InsideFunction)
{
    EXPECT_EQ(containingFunc(0xfffff, 0), 0xe7fff);
}

TEST_F(BitUnionData, BitfieldToBitfieldAssignment)
{
    SixtyFour otherSixtyFour = 0;
    sixtyFour.bit2 = 1;
    otherSixtyFour.byte6.bit41 = sixtyFour.bit2;
    EXPECT_EQ(otherSixtyFour, 0x20000000000);
    otherSixtyFour.bit2 = sixtyFour.bit2;
    EXPECT_EQ(otherSixtyFour, 0x20000000004);
}

TEST_F(BitUnionData, Operators)
{
    SixtyFour otherSixtyFour = 0x4;
    sixtyFour = otherSixtyFour;
    EXPECT_EQ(sixtyFour, 0x4);
    sixtyFour = 0;
    EXPECT_TRUE(sixtyFour < otherSixtyFour);
    EXPECT_TRUE(otherSixtyFour > sixtyFour);
    EXPECT_TRUE(sixtyFour != otherSixtyFour);
    sixtyFour = otherSixtyFour;
    EXPECT_TRUE(sixtyFour == otherSixtyFour);
}

TEST_F(BitUnionData, Custom)
{
    EXPECT_EQ(split, 0);
    split.split = 0xfff;
    EXPECT_EQ(split, 0xf0f0);
    EXPECT_EQ((uint64_t)split.split, 0xff);
}

TEST_F(BitUnionData, Templating)
{
    sixtyFour = 0xff;
    EXPECT_EQ(templatedFunction(sixtyFour), 0xff);
    EXPECT_EQ(templatedFunction((uint64_t)sixtyFour), 0);

    BitUnion(uint64_t, Dummy64)
    EndBitUnion(Dummy64);

    BitUnion(uint32_t, Dummy32)
    EndBitUnion(Dummy32);

    bool is64;
    is64 = std::is_same<BitUnionBaseType<Dummy64>, uint64_t>::value;
    EXPECT_TRUE(is64);
    is64 = std::is_same<BitUnionBaseType<Dummy32>, uint64_t>::value;
    EXPECT_FALSE(is64);
}
