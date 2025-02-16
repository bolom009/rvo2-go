package rvo2_go

import (
	"math"
)

const (
	MaxLeafSize int = 10
)

// KdTree defines k-D trees for agents and static obstacles in the simulation.
type KdTree struct {
	ObstacleTree *ObstacleTreeNode
	AgentTree    []*AgentTreeNode
	Agents       []*Agent
}

// AgentTreeNode defines an agent k-D tree node.
type AgentTreeNode struct {
	Begin int
	End   int
	Left  int
	Right int
	MaxX  float32
	MaxY  float32
	MinX  float32
	MinY  float32
}

// ObstacleTreeNode defines an obstacle k-D tree node.
type ObstacleTreeNode struct {
	Left     *ObstacleTreeNode
	Right    *ObstacleTreeNode
	Obstacle *Obstacle
}

// NewAgentTreeNode constructs an agent k-D tree node instance.
func NewAgentTreeNode() *AgentTreeNode {
	a := &AgentTreeNode{
		Begin: 0,
		End:   0,
		Left:  0,
		Right: 0,
		MaxX:  0,
		MaxY:  0,
		MinX:  0,
		MinY:  0,
	}
	return a
}

// NewObstacleTreeNode constructs an obstacle k-D tree node instance.
func NewObstacleTreeNode() *ObstacleTreeNode {
	o := &ObstacleTreeNode{
		Left:     nil,
		Right:    nil,
		Obstacle: NewEmptyObstacle(),
	}
	return o
}

// NewKdTree constructs a k-D tree instance.
func NewKdTree() *KdTree {
	k := &KdTree{
		ObstacleTree: nil,
	}
	return k
}

// BuildAgentTree builds an agent k-D tree.
func (kt *KdTree) BuildAgentTree() {
	kt.Agents = nil
	if len(kt.Agents) < len(Sim.Agents) {
		for i := len(kt.Agents); i < len(Sim.Agents); i++ {
			// skip disabled agents in building agents tree
			if !Sim.Agents[i].active {
				continue
			}

			kt.Agents = append(kt.Agents, Sim.Agents[i])
		}

		// AgentTreeを2*len(kt.Agents)-1で初期化
		kt.AgentTree = make([]*AgentTreeNode, 2*len(kt.Agents)-1)
		for i := 0; i < 2*len(kt.Agents)-1; i++ {
			kt.AgentTree[i] = NewAgentTreeNode()
		}
	}

	if len(kt.Agents) != 0 {
		kt.BuildAgentTreeRecursive(0, len(kt.Agents), 0)
	}
}

// BuildAgentTreeRecursive recursive function to build an agent k-D tree.
func (kt *KdTree) BuildAgentTreeRecursive(begin int, end int, node int) {

	kt.AgentTree[node].Begin = begin
	kt.AgentTree[node].End = end
	kt.AgentTree[node].MinX = kt.Agents[begin].Position.X
	kt.AgentTree[node].MaxX = kt.Agents[begin].Position.X
	kt.AgentTree[node].MinY = kt.Agents[begin].Position.Y
	kt.AgentTree[node].MaxY = kt.Agents[begin].Position.Y

	// i-1番目までのAgentとi番目のAgentのポジションを比較
	for i := begin + 1; i < end; i++ {
		kt.AgentTree[node].MaxX = float32(maxFn(float64(kt.AgentTree[node].MaxX), float64(kt.Agents[i].Position.X)))
		kt.AgentTree[node].MinX = float32(minFn(float64(kt.AgentTree[node].MinX), float64(kt.Agents[i].Position.X)))
		kt.AgentTree[node].MaxY = float32(maxFn(float64(kt.AgentTree[node].MaxY), float64(kt.Agents[i].Position.Y)))
		kt.AgentTree[node].MinY = float32(minFn(float64(kt.AgentTree[node].MinY), float64(kt.Agents[i].Position.Y)))
	}
	if end-begin > MaxLeafSize {
		isVertical := kt.AgentTree[node].MaxX-kt.AgentTree[node].MinX > kt.AgentTree[node].MaxY-kt.AgentTree[node].MinY
		left := begin
		right := end
		var splitValue, leftPosition, rightPosition float32

		if isVertical {
			splitValue = 0.5 * (kt.AgentTree[node].MaxX + kt.AgentTree[node].MinX)
		} else {
			splitValue = 0.5 * (kt.AgentTree[node].MaxY + kt.AgentTree[node].MinY)
		}

		for {
			if left < right {
				for {

					if left < right {
						if isVertical {
							leftPosition = kt.Agents[left].Position.X
						} else {
							leftPosition = kt.Agents[left].Position.Y
						}

						if leftPosition < splitValue {
							left++
						} else {
							break
						}

					} else {
						break
					}
				}

				for {
					if right > left {
						if isVertical {
							rightPosition = kt.Agents[right-1].Position.X
						} else {
							rightPosition = kt.Agents[right-1].Position.Y
						}
						if rightPosition >= splitValue {
							right--
						} else {
							break
						}
					} else {
						break
					}
				}

				if left < right {
					// Swap array
					t := kt.Agents[left]
					kt.Agents[left] = kt.Agents[right-1]
					kt.Agents[right-1] = t

					left++
					right--
				}
			} else {
				break
			}

		}

		if left == begin {
			left++
			right++
		}

		kt.AgentTree[node].Left = node + 1
		kt.AgentTree[node].Right = node + 2*(left-begin)

		kt.BuildAgentTreeRecursive(begin, left, kt.AgentTree[node].Left)
		kt.BuildAgentTreeRecursive(left, end, kt.AgentTree[node].Right)
	}
}

// BuildObstacleTree builds an obstacle k-D tree.
func (kt *KdTree) BuildObstacleTree() {
	kt.DeleteObstacleTree(kt.ObstacleTree)

	obstacles := make([]*Obstacle, len(Sim.ObstacleVertices))
	for i := 0; i < len(Sim.ObstacleVertices); i++ {
		obstacles[i] = Sim.ObstacleVertices[i]
	}

	kt.ObstacleTree = kt.BuildObstacleTreeRecursive(obstacles)
}

// BuildObstacleTreeRecursive recursive function to build an obstacle k-D tree.
func (kt *KdTree) BuildObstacleTreeRecursive(obstacles []*Obstacle) *ObstacleTreeNode {
	if len(obstacles) == 0 {
		return nil
	} else {
		node := NewObstacleTreeNode()

		optimalSplit := 0
		minLeft := len(obstacles)
		minRight := len(obstacles)

		// Compute optimal split node.
		for i := 0; i < len(obstacles); i++ {
			leftSize := 0
			rightSize := 0

			var obstacleI1, obstacleI2 *Obstacle
			obstacleI1 = obstacles[i]
			obstacleI2 = obstacleI1.NextObstacle

			for j := 0; j < len(obstacles); j++ {
				if i == j {
					continue
				}
				var obstacleJ1, obstacleJ2 *Obstacle
				obstacleJ1 = obstacles[j]
				obstacleJ2 = obstacleJ1.NextObstacle

				j1LeftOfI := LeftOf(obstacleI1.Point, obstacleI2.Point, obstacleJ1.Point)
				j2LeftOfI := LeftOf(obstacleI1.Point, obstacleI2.Point, obstacleJ2.Point)

				if j1LeftOfI >= -RvoEpsilon && j2LeftOfI >= -RvoEpsilon {
					leftSize++
				} else if j1LeftOfI <= RvoEpsilon && j2LeftOfI <= RvoEpsilon {
					rightSize++
				} else {
					leftSize++
					rightSize++
				}

				if maxFn(float64(leftSize), float64(rightSize)) > maxFn(float64(minLeft), float64(minRight)) || (maxFn(float64(leftSize), float64(rightSize)) == maxFn(float64(minLeft), float64(minRight)) && minFn(float64(leftSize), float64(rightSize)) >= minFn(float64(minLeft), float64(minRight))) {
					break
				}

			}

			if maxFn(float64(leftSize), float64(rightSize)) < maxFn(float64(minLeft), float64(minRight)) || (maxFn(float64(leftSize), float64(rightSize)) == maxFn(float64(minLeft), float64(minRight)) && minFn(float64(leftSize), float64(rightSize)) < minFn(float64(minLeft), float64(minRight))) {
				minLeft = leftSize
				minRight = rightSize
				optimalSplit = i
			}

		}

		// Build split node.
		var leftObstacles, rightObstacles []*Obstacle
		leftObstacles = make([]*Obstacle, minLeft)
		rightObstacles = make([]*Obstacle, minRight)

		leftCounter := 0
		rightCounter := 0
		i := optimalSplit

		var obstacleI1, obstacleI2 *Obstacle
		obstacleI1 = obstacles[i]
		obstacleI2 = obstacleI1.NextObstacle

		for j := 0; j < len(obstacles); j++ {
			if i == j {
				continue
			}

			var obstacleJ1, obstacleJ2 *Obstacle
			obstacleJ1 = obstacles[j]
			obstacleJ2 = obstacleJ1.NextObstacle

			var j1LeftOfI, j2LeftOfI float32
			j1LeftOfI = LeftOf(obstacleI1.Point, obstacleI2.Point, obstacleJ1.Point)
			j2LeftOfI = LeftOf(obstacleI1.Point, obstacleI2.Point, obstacleJ2.Point)

			if j1LeftOfI >= -RvoEpsilon && j2LeftOfI >= -RvoEpsilon {
				leftObstacles[leftCounter] = obstacles[j]
				leftCounter++
			} else if j1LeftOfI <= RvoEpsilon && j2LeftOfI <= RvoEpsilon {
				rightObstacles[rightCounter] = obstacles[j]
				rightCounter++
			} else {
				var t float32
				t = Det(Sub(obstacleI2.Point, obstacleI1.Point), Sub(obstacleJ1.Point, obstacleI1.Point)) / Det(Sub(obstacleI2.Point, obstacleI1.Point), Sub(obstacleJ1.Point, obstacleJ2.Point))

				var splitPoint *Vector2
				splitPoint = Add(obstacleJ1.Point, MulOne(Sub(obstacleJ2.Point, obstacleJ1.Point), t))

				newObstacle := NewEmptyObstacle()
				newObstacle.Point = splitPoint
				newObstacle.PrevObstacle = obstacleJ1
				newObstacle.NextObstacle = obstacleJ2
				newObstacle.IsConvex = true
				newObstacle.UnitDir = obstacleJ1.UnitDir
				newObstacle.ID = len(Sim.ObstacleVertices)

				Sim.ObstacleVertices = append(Sim.ObstacleVertices, newObstacle)

				obstacleJ1.NextObstacle = newObstacle
				obstacleJ2.PrevObstacle = newObstacle

				if j1LeftOfI > 0.0 {
					leftObstacles[leftCounter] = obstacleJ1
					rightObstacles[rightCounter] = newObstacle
					leftCounter++
					rightCounter++
				} else {
					rightObstacles[rightCounter] = obstacleJ1
					leftObstacles[leftCounter] = newObstacle
					leftCounter++
					rightCounter++
				}

			}

		}
		node.Obstacle = obstacleI1
		node.Left = kt.BuildObstacleTreeRecursive(leftObstacles)
		node.Right = kt.BuildObstacleTreeRecursive(rightObstacles)

		return node
	}
}

// ComputeAgentNeighbors computes the agent neighbors of the specified agent.
func (kt *KdTree) ComputeAgentNeighbors(agent *Agent, rangeSq float32) {
	kt.QueryAgentTreeRecursive(agent, rangeSq, 0)
}

// ComputeObstacleNeighbors computes the obstacle neighbors of the specified agent.
func (kt *KdTree) ComputeObstacleNeighbors(agent *Agent, rangeSq float32) {
	kt.QueryObstacleTreeRecursive(agent, rangeSq, kt.ObstacleTree)
}

// DeleteObstacleTree deletes the specified obstacle tree node.
func (kt *KdTree) DeleteObstacleTree(node *ObstacleTreeNode) {
	if node != nil {
		kt.DeleteObstacleTree(node.Left)
		kt.DeleteObstacleTree(node.Right)
	}
}

// QueryAgentTreeRecursive recursive function to compute the neighbors of the specified agent.
func (kt *KdTree) QueryAgentTreeRecursive(agent *Agent, rangeSq float32, node int) {
	if kt.AgentTree[node].End-kt.AgentTree[node].Begin <= MaxLeafSize {
		for i := kt.AgentTree[node].Begin; i < kt.AgentTree[node].End; i++ {
			agent.InsertAgentNeighbor(kt.Agents[i], &rangeSq)
		}
	} else {
		// calc left dist sq
		distLeftMinX := maxFn(0, float64(kt.AgentTree[kt.AgentTree[node].Left].MinX-agent.Position.X))
		distLeftMaxX := maxFn(0, float64(agent.Position.X-kt.AgentTree[kt.AgentTree[node].Left].MaxX))
		distLeftMinY := maxFn(0, float64(kt.AgentTree[kt.AgentTree[node].Left].MinY-agent.Position.Y))
		distLeftMaxY := maxFn(0, float64(agent.Position.Y-kt.AgentTree[kt.AgentTree[node].Left].MaxY))
		distSqLeft := float32(distLeftMinX*distLeftMinX + distLeftMaxX*distLeftMaxX +
			distLeftMinY*distLeftMinY + distLeftMaxY*distLeftMaxY)

		// calc right dist sq
		distRightMinX := maxFn(0, float64(kt.AgentTree[kt.AgentTree[node].Right].MinX-agent.Position.X))
		distRightMaxX := maxFn(0, float64(agent.Position.X-kt.AgentTree[kt.AgentTree[node].Right].MaxX))
		distRightMinY := maxFn(0, float64(kt.AgentTree[kt.AgentTree[node].Right].MinY-agent.Position.Y))
		distRightMaxY := maxFn(0, float64(agent.Position.Y-kt.AgentTree[kt.AgentTree[node].Right].MaxY))
		distSqRight := float32(distRightMinX*distRightMinX + distRightMaxX*distRightMaxX +
			distRightMinY*distRightMinY + distRightMaxY*distRightMaxY)

		if distSqLeft < distSqRight {
			if distSqLeft < rangeSq {
				kt.QueryAgentTreeRecursive(agent, rangeSq, kt.AgentTree[node].Left)

				if distSqRight < rangeSq {
					kt.QueryAgentTreeRecursive(agent, rangeSq, kt.AgentTree[node].Right)
				}
			}
		} else {
			if distSqRight < rangeSq {
				kt.QueryAgentTreeRecursive(agent, rangeSq, kt.AgentTree[node].Right)

				if distSqLeft < rangeSq {
					kt.QueryAgentTreeRecursive(agent, rangeSq, kt.AgentTree[node].Left)
				}
			}
		}
	}
}

// QueryObstacleTreeRecursive recursive function to compute the neighbors of the specified obstacle.
func (kt *KdTree) QueryObstacleTreeRecursive(agent *Agent, rangeSq float32, node *ObstacleTreeNode) {
	if node != nil {
		obstacle1 := node.Obstacle
		obstacle2 := obstacle1.NextObstacle

		agentLeftOfLine := LeftOf(obstacle1.Point, obstacle2.Point, agent.Position)

		var tNode *ObstacleTreeNode
		if agentLeftOfLine >= 0 {
			tNode = node.Left
		} else {
			tNode = node.Right
		}

		kt.QueryObstacleTreeRecursive(agent, rangeSq, tNode)

		dblAgentLeftOfLine := agentLeftOfLine * agentLeftOfLine
		v21x := obstacle2.Point.X - obstacle1.Point.X
		v21y := obstacle2.Point.Y - obstacle1.Point.Y

		dSqr := v21x*v21x + v21y*v21y

		distSqLine := dblAgentLeftOfLine / dSqr
		//distSqLine := dblAgentLeftOfLine / Sqr(Sub(obstacle2.Point, obstacle1.Point))

		if distSqLine < rangeSq {
			if agentLeftOfLine < 0 {
				agent.InsertObstacleNeighbor(node.Obstacle, rangeSq)
			}

			// Caution! Modified From RVO2  Original Library
			/* Try other side of line. */
			/*var t2Node *ObstacleTreeNode
			if agentLeftOfLine >= 0 {
				t2Node = node.Right
			} else {
				t2Node = node.Left
			}
			kt.QueryObstacleTreeRecursive(agent, rangeSq, t2Node)*/

		}

		/* Try other side of line. */
		var t2Node *ObstacleTreeNode
		if agentLeftOfLine >= 0 {
			t2Node = node.Right
		} else {
			t2Node = node.Left
		}
		kt.QueryObstacleTreeRecursive(agent, rangeSq, t2Node)
	}
}

// QueryVisibility queries the visibility between two points within a specified radius.
func (kt *KdTree) QueryVisibility(q1 *Vector2, q2 *Vector2, radius float32) bool {
	result := kt.QueryVisibilityRecursive(q1, q2, radius, kt.ObstacleTree)
	return result
}

// QueryVisibilityRecursive recursive function to query the visibility between two points within a specified radius.
func (kt *KdTree) QueryVisibilityRecursive(q1 *Vector2, q2 *Vector2, radius float32, node *ObstacleTreeNode) bool {
	if node == nil {
		return true
	} else {
		var obstacle1, obstacle2 *Obstacle
		obstacle1 = node.Obstacle
		obstacle2 = obstacle1.NextObstacle

		var q1LeftOfI, q2LeftOfI, invLengthI float32
		q1LeftOfI = LeftOf(obstacle1.Point, obstacle2.Point, q1)
		q2LeftOfI = LeftOf(obstacle1.Point, obstacle2.Point, q2)
		invLengthI = 1.0 / Sqr(Sub(obstacle2.Point, obstacle1.Point))

		if q1LeftOfI >= 0 && q2LeftOfI >= 0 {
			return kt.QueryVisibilityRecursive(q1, q2, radius, node.Left) && ((float32(math.Pow(float64(q1LeftOfI), 2))*invLengthI >= float32(math.Pow(float64(radius), 2)) && float32(math.Pow(float64(q2LeftOfI), 2))*invLengthI >= float32(math.Pow(float64(radius), 2))) || kt.QueryVisibilityRecursive(q1, q2, radius, node.Right))
		} else if q1LeftOfI <= 0 && q2LeftOfI <= 0 {
			return kt.QueryVisibilityRecursive(q1, q2, radius, node.Right) && ((float32(math.Pow(float64(q1LeftOfI), 2))*invLengthI >= float32(math.Pow(float64(radius), 2)) && float32(math.Pow(float64(q2LeftOfI), 2))*invLengthI >= float32(math.Pow(float64(radius), 2))) || kt.QueryVisibilityRecursive(q1, q2, radius, node.Left))
		} else if q1LeftOfI >= 0 && q2LeftOfI <= 0 {
			/* One can see through obstacle from Left to Right. */
			return kt.QueryVisibilityRecursive(q1, q2, radius, node.Left) && kt.QueryVisibilityRecursive(q1, q2, radius, node.Right)
		} else {
			var point1LeftOfQ, point2LeftOfQ, invLengthQ float32
			point1LeftOfQ = LeftOf(q1, q2, obstacle1.Point)
			point2LeftOfQ = LeftOf(q1, q2, obstacle2.Point)
			invLengthQ = 1.0 / Sqr(Sub(q2, q1))

			return point1LeftOfQ*point2LeftOfQ >= 0 && float32(math.Pow(float64(point1LeftOfQ), 2))*invLengthQ > float32(math.Pow(float64(radius), 2)) && float32(math.Pow(float64(point2LeftOfQ), 2))*invLengthQ > float32(math.Pow(float64(radius), 2)) && kt.QueryVisibilityRecursive(q1, q2, radius, node.Left) && kt.QueryVisibilityRecursive(q1, q2, radius, node.Right)
		}
	}
}

func maxFn(x, y float64) float64 {
	if x > y {
		return x
	}
	return y
}

func minFn(x, y float64) float64 {
	if x < y {
		return x
	}
	return y
}
