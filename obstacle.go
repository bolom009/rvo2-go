package rvo2_go

// Obstacle :
type Obstacle struct {
	ID           int
	IsConvex     bool
	NextObstacle *Obstacle
	PrevObstacle *Obstacle
	Point        *Vector2
	UnitDir      *Vector2
}

// NewEmptyObstacle : To create new obstacle object
func NewEmptyObstacle() *Obstacle {
	o := &Obstacle{
		ID:           0,
		IsConvex:     false,
		NextObstacle: nil,
		PrevObstacle: nil,
	}
	return o
}
