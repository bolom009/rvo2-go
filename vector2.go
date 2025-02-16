package rvo2_go

import (
	"math"
)

// Vector2 :
type Vector2 struct {
	X float32
	Y float32
}

// NewVector2 :
func NewVector2(x float32, y float32) *Vector2 {
	v := &Vector2{
		X: x,
		Y: y,
	}
	return v
}

// Flip :
func Flip(vec *Vector2) *Vector2 {
	return &Vector2{X: -vec.X, Y: -vec.Y}
}

// Sub:
func Sub(vec1 *Vector2, vec2 *Vector2) *Vector2 {
	return &Vector2{X: vec1.X - vec2.X, Y: vec1.Y - vec2.Y}
}

// Add :
func Add(vec1 *Vector2, vec2 *Vector2) *Vector2 {
	return &Vector2{X: vec1.X + vec2.X, Y: vec1.Y + vec2.Y}
}

// Mul :
func Mul(vec1 *Vector2, vec2 *Vector2) float32 {
	return vec1.X*vec2.X + vec1.Y*vec2.Y
}

// MulOne :
func MulOne(vec *Vector2, s float32) *Vector2 {
	return &Vector2{X: vec.X * s, Y: vec.Y * s}
}

// Div :
func Div(vec *Vector2, s float32) *Vector2 {
	return &Vector2{X: vec.X / s, Y: vec.Y / s}
}

// Equal :
func Equal(vec1 *Vector2, vec2 *Vector2) bool {
	return vec1.X == vec2.X && vec1.Y == vec2.Y
}

// NotEqual :
func NotEqual(vec1 *Vector2, vec2 *Vector2) bool {
	return vec1.X != vec2.X || vec1.Y != vec2.Y
}

// MulSum :
func MulSum(vec *Vector2, s float32) *Vector2 {
	return Add(vec, MulOne(vec, s))
}

// DivSum :
func DivSum(vec *Vector2, s float32) *Vector2 {

	return Add(vec, Div(vec, s))
}

// AddSum :
func AddSum(vec1 *Vector2, vec2 *Vector2) *Vector2 {
	return Add(vec1, Add(vec1, vec2))
}

// SubSum :
func SubSum(vec1 *Vector2, vec2 *Vector2) *Vector2 {
	return Add(vec1, Sub(vec1, vec2))
}

// Sqr :
func Sqr(vec *Vector2) float32 {
	return Mul(vec, vec)
}

// Abs :
func Abs(vec *Vector2) float32 {
	return float32(math.Sqrt(float64(Mul(vec, vec))))
}

// Normalize :
func Normalize(vec *Vector2) *Vector2 {
	return Div(vec, Abs(vec))
}

// Det :
func Det(vec1 *Vector2, vec2 *Vector2) float32 {
	return vec1.X*vec2.Y - vec1.Y*vec2.X
}

// LeftOf :
func LeftOf(vec1 *Vector2, vec2 *Vector2, vec3 *Vector2) float32 {
	v13x := vec1.X - vec3.X
	v13y := vec1.Y - vec3.Y

	v21x := vec2.X - vec1.X
	v21y := vec2.Y - vec1.Y

	return v13x*v21y - v13y*v21x
}

// DistSqPointLineSegment :
func DistSqPointLineSegment(vec1 *Vector2, vec2 *Vector2, vec3 *Vector2) float32 {
	v31x := vec3.X - vec1.X
	v31y := vec3.Y - vec1.Y

	v21x := vec2.X - vec1.X
	v21y := vec2.Y - vec1.Y

	// vec1.X*vec2.X + vec1.Y*vec2.Y
	mul := v31x*v21x + v31y*v21y
	dSqr := v21x*v21x + v21y*v21y

	//r := Mul(Sub(vec3, vec1), Sub(vec2, vec1)) / Sqr(Sub(vec2, vec1))
	r := mul / dSqr

	if r < 0 {
		return v31x*v31x + v31y*v31y
		//return Sqr(Sub(vec3, vec1))
	} else if r > 1 {
		v32x := vec3.X - vec2.X
		v32y := vec3.Y - vec2.Y
		return v32x*v32x + v32y*v32y
		//return Sqr(Sub(vec3, vec2))
	} else {
		v21xScalar := v21x * r
		v21yScalar := v21y * r

		addVx := v21xScalar + vec1.X
		addVy := v21yScalar + vec1.Y

		v2x := vec3.X - addVx
		v2y := vec3.Y - addVy

		return v2x*v2x + v2y*v2y
		//return Sqr(
		//	Sub(
		//		vec3,
		//		Add(vec1,
		//			MulOne(Sub(vec2, vec1), r),
		//		),
		//	),
		//)
	}
}
