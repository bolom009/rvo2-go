package rvo2_go

// Obstacle defines static obstacles in the simulation.
type Obstacle struct {
	ID           int
	IsConvex     bool
	NextObstacle *Obstacle
	PrevObstacle *Obstacle
	Point        *Vector2
	UnitDir      *Vector2
}

// NewEmptyObstacle constructs a static obstacle instance.
func NewEmptyObstacle() *Obstacle {
	o := &Obstacle{
		ID:           0,
		IsConvex:     false,
		NextObstacle: nil,
		PrevObstacle: nil,
	}
	return o
}
