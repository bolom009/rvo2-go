package rvo2_go

import (
	"fmt"
	"math"
	"testing"
)

func Benchmark_MulOne(b *testing.B) {
	vec := NewVector2(21.52, 55.178)
	scalar := float32(7.0)

	b.ReportAllocs()
	b.ResetTimer()

	for i := 0; i < b.N; i++ {
		MulOne(vec, scalar)
	}
}

// Benchmark_Pow-16    	        99041770	        12.16 ns/op	       0 B/op	       0 allocs/op
// Benchmark_PowInline-16    	1000000000	        0.2204 ns/op	       0 B/op	       0 allocs/op
func Benchmark_Pow(b *testing.B) {
	fmt.Println(math.Pow(math.Pi, 2))

	b.ReportAllocs()
	b.ResetTimer()

	for i := 0; i < b.N; i++ {
		_ = math.Pow(math.Pi, 2)
	}
}

func Benchmark_PowInline(b *testing.B) {
	fmt.Println(math.Pi * math.Pi)

	b.ReportAllocs()
	b.ResetTimer()

	for i := 0; i < b.N; i++ {
		_ = math.Pi * math.Pi
	}
}

// Benchmark_Max-16    	599596171	         2.070 ns/op	       0 B/op	       0 allocs/op
// Benchmark_MaxInline-16    	1000000000	         0.2251 ns/op	       0 B/op	       0 allocs/op
func Benchmark_Max(b *testing.B) {
	a := float64(1)
	c := float64(2)

	b.ReportAllocs()
	b.ResetTimer()

	for i := 0; i < b.N; i++ {
		_ = math.Max(a, c)
	}
}

func Benchmark_MaxInline(b *testing.B) {
	a := float64(1)
	c := float64(2)

	b.ReportAllocs()
	b.ResetTimer()

	for i := 0; i < b.N; i++ {
		_ = maxFn(a, c)
	}
}
