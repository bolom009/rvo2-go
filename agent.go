package rvo2_go

import (
	"math"
)

const RvoEpsilon float32 = 0.00001 //A sufficiently small positive number.

// Agent represent structure of rvo agent
type Agent struct {
	ID                uint16
	Position          *Vector2
	Radius            float32
	TimeHorizon       float32
	TimeHorizonObst   float32
	Velocity          *Vector2
	PrefVelocity      *Vector2
	NewVelocity       *Vector2
	MaxNeighbors      uint16
	NeighborDist      float32
	MaxSpeed          float32
	ObstacleNeighbors []*ObstacleNeighbor
	AgentNeighbors    []*AgentNeighbor
	OrcaLines         []*Line
	Goal              *Vector2
	// active agent include in rvo simulation
	active bool
}

// ObstacleNeighbor represent near rvo obstacle
type ObstacleNeighbor struct {
	DistSq   float32
	Obstacle *Obstacle
}

// AgentNeighbor represent near rvo agent
type AgentNeighbor struct {
	DistSq float32
	Agent  *Agent
}

// Line :
type Line struct {
	Point     *Vector2
	Direction *Vector2
}

// NewEmptyAgent constructor for creating an empty agent
func NewEmptyAgent() *Agent {
	a := &Agent{
		ID:              0,
		Radius:          0,
		TimeHorizon:     0,
		TimeHorizonObst: 0,
		MaxNeighbors:    0,
		NeighborDist:    0,
		MaxSpeed:        0,
		active:          true,
	}
	return a
}

// NewAgent constructor for creating an agent with parameters
func NewAgent(id uint16, position *Vector2, radius, timeHorizon, timeHorizonObst float32, velocity, newVelocity, prefVelocity *Vector2, maxNeighbors uint16, neighborDist, maxSpeed float32, obstacleNeighbors []*ObstacleNeighbor) *Agent {
	a := &Agent{
		ID:                id,
		Position:          position,
		Radius:            radius,
		TimeHorizon:       timeHorizon,
		TimeHorizonObst:   timeHorizonObst,
		Velocity:          velocity,
		PrefVelocity:      prefVelocity,
		NewVelocity:       newVelocity,
		MaxNeighbors:      maxNeighbors,
		NeighborDist:      neighborDist,
		MaxSpeed:          maxSpeed,
		ObstacleNeighbors: obstacleNeighbors,
		active:            true,
	}
	return a
}

// ComputeNeighbors compute agent neighbors
func (a *Agent) ComputeNeighbors() {
	a.ObstacleNeighbors = make([]*ObstacleNeighbor, 0)
	rangeSq := float32(math.Pow(float64(a.TimeHorizonObst*a.MaxSpeed+a.Radius), 2))

	Sim.KdTree.ComputeObstacleNeighbors(a, rangeSq)

	a.AgentNeighbors = make([]*AgentNeighbor, 0)

	if a.MaxNeighbors > 0 {
		rangeSq = float32(math.Pow(float64(a.NeighborDist), 2))
		Sim.KdTree.ComputeAgentNeighbors(a, rangeSq)
	}
}

// ComputeNewVelocity compute agent velocity
func (a *Agent) ComputeNewVelocity() {
	a.OrcaLines = make([]*Line, 0, len(a.ObstacleNeighbors))

	invTimeHorizonObst := float32(1.0) / a.TimeHorizonObst

	/* Create obstacle ORCA lines. */
	for i := 0; i < len(a.ObstacleNeighbors); i++ {

		var obstacle1, obstacle2 *Obstacle
		obstacle1 = a.ObstacleNeighbors[i].Obstacle
		obstacle2 = obstacle1.NextObstacle

		var relativePosition1, relativePosition2 *Vector2
		relativePosition1 = Sub(obstacle1.Point, a.Position)
		relativePosition2 = Sub(obstacle2.Point, a.Position)

		/*
		 * Check if velocity obstacle of obstacle is already taken care of by
		 * previously constructed obstacle ORCA lines.
		 */
		var alreadyCovered bool = false

		for j := 0; j < len(a.OrcaLines); j++ {
			if Det(Sub(MulOne(relativePosition1, invTimeHorizonObst), a.OrcaLines[j].Point), a.OrcaLines[j].Direction)-invTimeHorizonObst*a.Radius >= -RvoEpsilon && Det(Sub(MulOne(relativePosition2, invTimeHorizonObst), a.OrcaLines[j].Point), a.OrcaLines[j].Direction)-invTimeHorizonObst*a.Radius >= -RvoEpsilon {
				alreadyCovered = true
				break
			}
		}

		if alreadyCovered {
			continue
		}

		/* Not yet covered. Check for collisions. */

		var distSq1, distSq2, radiusSq float32
		distSq1 = Sqr(relativePosition1)
		distSq2 = Sqr(relativePosition2)

		radiusSq = float32(math.Pow(float64(a.Radius), 2))

		var obstacleVector *Vector2
		obstacleVector = Sub(obstacle2.Point, obstacle1.Point)
		var s, distSqLine float32
		s = Mul(Flip(relativePosition1), obstacleVector) / Sqr(obstacleVector)
		distSqLine = Sqr(Sub(Flip(relativePosition1), MulOne(obstacleVector, s)))

		var line Line
		if s < 0 && distSq1 <= radiusSq {
			/* Collision with left vertex. Ignore if non-convex. */
			if obstacle1.IsConvex {
				line.Point = NewVector2(0, 0)
				line.Direction = Normalize(NewVector2(-relativePosition1.Y, relativePosition1.X))
				a.OrcaLines = append(a.OrcaLines, &line)
			}

			continue
		} else if s > 1 && distSq2 <= radiusSq {
			/* Collision with right vertex. Ignore if non-convex
			 * or if it will be taken care of by neighoring obstace */
			if obstacle2.IsConvex && Det(relativePosition2, obstacle2.UnitDir) >= 0 {
				line.Point = NewVector2(0, 0)
				line.Direction = Normalize(NewVector2(-relativePosition2.Y, relativePosition2.X))
				a.OrcaLines = append(a.OrcaLines, &line)
			}

			continue
		} else if s >= 0 && s < 1 && distSqLine <= radiusSq {
			/* Collision with obstacle segment. */

			line.Point = NewVector2(0, 0)
			line.Direction = Flip(obstacle1.UnitDir)
			a.OrcaLines = append(a.OrcaLines, &line)
			continue
		}

		/*
		 * No collision.
		 * Compute legs. When obliquely viewed, both legs can come from a single
		 * vertex. Legs extend cut-off line when nonconvex vertex.
		 */

		var leftLegDirection, rightLegDirection *Vector2
		var leg1, leg2 float32

		if s < 0 && distSqLine <= radiusSq {
			/*
			 * Obstacle viewed obliquely so that left vertex
			 * defines velocity obstacle.
			 */
			if !obstacle1.IsConvex {
				/* Ignore obstacle. */
				continue
			}

			obstacle2 = obstacle1

			leg1 = float32(math.Sqrt(float64(distSq1 - radiusSq)))

			leftLegDirection = Div(NewVector2(relativePosition1.X*leg1-relativePosition1.Y*a.Radius, relativePosition1.X*a.Radius+relativePosition1.Y*leg1), distSq1)
			rightLegDirection = Div(NewVector2(relativePosition1.X*leg1+relativePosition1.Y*a.Radius, -relativePosition1.X*a.Radius+relativePosition1.Y*leg1), distSq1)
		} else if s > 1 && distSqLine <= radiusSq {
			/*
			 * Obstacle viewed obliquely so that
			 * right vertex defines velocity obstacle.
			 */
			if !obstacle2.IsConvex {
				/* Ignore obstacle. */
				continue
			}

			obstacle1 = obstacle2

			leg2 = float32(math.Sqrt(float64(distSq2 - radiusSq)))
			leftLegDirection = Div(NewVector2(relativePosition2.X*leg2-relativePosition2.Y*a.Radius, relativePosition2.X*a.Radius+relativePosition2.Y*leg2), distSq2)
			rightLegDirection = Div(NewVector2(relativePosition2.X*leg2+relativePosition2.Y*a.Radius, -relativePosition2.X*a.Radius+relativePosition2.Y*leg2), distSq2)
		} else {
			/* Usual situation. */
			if obstacle1.IsConvex {
				leg1 = float32(math.Sqrt(float64(distSq1 - radiusSq)))
				leftLegDirection = Div(NewVector2(relativePosition1.X*leg1-relativePosition1.Y*a.Radius, relativePosition1.X*a.Radius+relativePosition1.Y*leg1), distSq1)
			} else {
				/* Left vertex non-convex; left leg extends cut-off line. */
				leftLegDirection = Flip(obstacle1.UnitDir)
			}

			if obstacle2.IsConvex {
				leg2 = float32(math.Sqrt(float64(distSq2 - radiusSq)))
				rightLegDirection = Div(NewVector2(relativePosition2.X*leg2+relativePosition2.Y*a.Radius, -relativePosition2.X*a.Radius+relativePosition2.Y*leg2), distSq2)
			} else {
				/* Right vertex non-convex; right leg extends cut-off line. */
				rightLegDirection = obstacle1.UnitDir
			}
		}

		/*
		 * Legs can never point into neighboring edge when convex vertex,
		 * take cutoff-line of neighboring edge instead. If velocity projected on
		 * "foreign" leg, no constraint is added.
		 */

		var leftNeighbor *Obstacle
		leftNeighbor = obstacle1.PrevObstacle

		var isLeftLegForeign, isRightLegForeign bool
		isLeftLegForeign = false
		isRightLegForeign = false

		if obstacle1.IsConvex && Det(leftLegDirection, Flip(leftNeighbor.UnitDir)) >= 0 {
			/* Left leg points into obstacle. */
			leftLegDirection = Flip(leftNeighbor.UnitDir)
			isLeftLegForeign = true
		}

		if obstacle2.IsConvex && Det(rightLegDirection, obstacle2.UnitDir) <= 0 {
			/* Right leg points into obstacle. */
			rightLegDirection = obstacle2.UnitDir
			isRightLegForeign = true
		}

		/* Compute cut-off centers. */
		leftCutoff := MulOne(Sub(obstacle1.Point, a.Position), invTimeHorizonObst)
		rightCutoff := MulOne(Sub(obstacle2.Point, a.Position), invTimeHorizonObst)
		cutoffVec := Sub(rightCutoff, leftCutoff)

		/* Project current velocity on velocity obstacle. */

		/* Check if current velocity is projected on cutoff circles. */
		var t float32
		if obstacle1 == obstacle2 {
			t = 0.5
		} else {
			t = Mul(Sub(a.Velocity, leftCutoff), cutoffVec) / Sqr(cutoffVec)
		}
		var tLeft, tRight float32
		var unitW *Vector2
		tLeft = Mul(Sub(a.Velocity, leftCutoff), leftLegDirection)
		tRight = Mul(Sub(a.Velocity, rightCutoff), rightLegDirection)

		if (t < 0 && tLeft < 0) || (obstacle1 == obstacle2 && tLeft < 0 && tRight < 0) {
			/* Project on left cut-off circle. */
			unitW = Normalize(Sub(a.Velocity, leftCutoff))

			line.Direction = NewVector2(unitW.Y, -unitW.X)
			line.Point = Add(leftCutoff, MulOne(unitW, a.Radius*invTimeHorizonObst))
			a.OrcaLines = append(a.OrcaLines, &line)
			continue
		} else if t > 1 && tRight < 0 {
			/* Project on right cut-off circle. */
			unitW = Normalize(Sub(a.Velocity, rightCutoff))

			line.Direction = NewVector2(unitW.Y, -unitW.X)
			line.Point = Add(rightCutoff, MulOne(unitW, a.Radius*invTimeHorizonObst))
			a.OrcaLines = append(a.OrcaLines, &line)
			continue
		}

		/*
		 * Project on left leg, right leg, or cut-off line, whichever is closest
		 * to velocity.
		 */
		var distSqCutoff, distSqLeft, distSqRight float32
		if t < 0 || t > 1 || obstacle1 == obstacle2 {
			distSqCutoff = float32(math.Inf(0)) // positive infinity
		} else {
			distSqCutoff = Sqr(Sub(a.Velocity, Add(leftCutoff, MulOne(cutoffVec, t))))
		}
		if tLeft < 0 {
			distSqLeft = float32(math.Inf(0)) // positive infinity
		} else {
			distSqLeft = Sqr(Sub(a.Velocity, Add(leftCutoff, MulOne(leftLegDirection, tLeft))))
		}
		if tRight < 0 {
			distSqRight = float32(math.Inf(0)) // positive infinity
		} else {
			distSqRight = Sqr(Sub(a.Velocity, Add(rightCutoff, MulOne(rightLegDirection, tRight))))
		}

		if distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight {
			/* Project on cut-off line. */
			line.Direction = Flip(obstacle1.UnitDir)
			line.Point = Add(leftCutoff, MulOne(NewVector2(-line.Direction.Y, line.Direction.X), a.Radius*invTimeHorizonObst))
			a.OrcaLines = append(a.OrcaLines, &line)
			continue
		} else if distSqLeft <= distSqRight {
			/* Project on left leg. */
			if isLeftLegForeign {
				continue
			}

			line.Direction = leftLegDirection
			line.Point = Add(leftCutoff, MulOne(NewVector2(-line.Direction.Y, line.Direction.X), a.Radius*invTimeHorizonObst))
			a.OrcaLines = append(a.OrcaLines, &line)
			continue
		} else {
			/* Project on right leg. */
			if isRightLegForeign {
				continue
			}

			line.Direction = Flip(rightLegDirection)
			line.Point = Add(rightCutoff, MulOne(NewVector2(-line.Direction.Y, line.Direction.X), a.Radius*invTimeHorizonObst))
			a.OrcaLines = append(a.OrcaLines, &line)
			continue
		}
	}

	var numObstLines int
	numObstLines = len(a.OrcaLines)

	var invTimeHorizon float32
	invTimeHorizon = float32(1) / a.TimeHorizon

	/* Create agent ORCA lines. */
	for i := 0; i < len(a.AgentNeighbors); i++ {
		// otherが違う
		var other *Agent
		other = a.AgentNeighbors[i].Agent

		var relativePosition, relativeVelocity *Vector2
		relativePosition = Sub(other.Position, a.Position)
		relativeVelocity = Sub(a.Velocity, other.Velocity)

		var distSq, combinedRadius, combinedRadiusSq float32
		distSq = Sqr(relativePosition)
		combinedRadius = a.Radius + other.Radius
		combinedRadiusSq = float32(math.Pow(float64(combinedRadius), 2))

		var line Line
		var u, w, unitW *Vector2
		var wLengthSq, wLength float32
		if distSq > combinedRadiusSq {
			/* No collision. */
			w = Sub(relativeVelocity, MulOne(relativePosition, invTimeHorizon))
			/* Vector from cutoff center to relative velocity. */

			var dotProduct1, dotProduct2, leg float32
			wLengthSq = Sqr(w)
			dotProduct1 = Mul(w, relativePosition)

			if dotProduct1 < 0 && float32(math.Pow(float64(dotProduct1), 2)) > combinedRadiusSq*wLengthSq {
				/* Project on cut-off circle. */
				wLength = float32(math.Sqrt(float64(wLengthSq)))
				unitW = Div(w, wLength)

				line.Direction = NewVector2(unitW.Y, -unitW.X)
				u = MulOne(unitW, combinedRadius*invTimeHorizon-wLength)
			} else {
				/* Project on legs. */
				leg = float32(math.Sqrt(float64(distSq - combinedRadiusSq)))
				if Det(relativePosition, w) > 0 {
					/* Project on left leg. */
					line.Direction = Div(NewVector2(relativePosition.X*leg-relativePosition.Y*combinedRadius, relativePosition.X*combinedRadius+relativePosition.Y*leg), distSq)
				} else {
					/* Project on right leg. */
					line.Direction = Flip(Div(NewVector2(relativePosition.X*leg+relativePosition.Y*combinedRadius, -relativePosition.X*combinedRadius+relativePosition.Y*leg), distSq))
				}

				dotProduct2 = Mul(relativeVelocity, line.Direction)

				u = Sub(MulOne(line.Direction, dotProduct2), relativeVelocity)
			}
		} else {
			/* Collision. Project on cut-off circle of time timeStep. */
			var invTimeStep float32
			invTimeStep = 1 / float32(Sim.TimeStep)

			/* Vector from cutoff center to relative velocity. */
			w = Sub(relativeVelocity, MulOne(relativePosition, invTimeStep))
			wLength = Abs(w)
			unitW = Div(w, wLength)

			line.Direction = NewVector2(unitW.Y, -unitW.X)
			u = MulOne(unitW, combinedRadius*invTimeStep-wLength)
		}

		line.Point = Add(a.Velocity, MulOne(u, 0.5))
		a.OrcaLines = append(a.OrcaLines, &line)
	}

	lineFail := a.LinearProgram2(a.OrcaLines, a.MaxSpeed, a.PrefVelocity, false)
	if lineFail < len(a.OrcaLines) {
		a.LinearProgram3(a.OrcaLines, numObstLines, lineFail, a.MaxSpeed)
	}
}

// InsertAgentNeighbor inserting agent neighbor
func (a *Agent) InsertAgentNeighbor(agent *Agent, rangeSq *float32) {
	if a != agent {
		distSq := Sqr(Sub(a.Position, agent.Position))

		// 2Agent間の距離が半径よりも近かった場合
		if distSq < *rangeSq {
			if uint16(len(a.AgentNeighbors)) < a.MaxNeighbors {
				a.AgentNeighbors = append(a.AgentNeighbors, &AgentNeighbor{DistSq: distSq, Agent: agent})
			}

			i := len(a.AgentNeighbors) - 1

			for {
				// TODO ACTIVE LOGIC
				if i != 0 && distSq < a.AgentNeighbors[i-1].DistSq {
					a.AgentNeighbors[i] = a.AgentNeighbors[i-1]
					i--
				} else {
					break
				}
			}

			a.AgentNeighbors[i] = &AgentNeighbor{
				DistSq: distSq,
				Agent:  agent,
			}

			if uint16(len(a.AgentNeighbors)) == a.MaxNeighbors {
				*rangeSq = a.AgentNeighbors[len(a.AgentNeighbors)-1].DistSq
			}
		}
	}
}

// InsertObstacleNeighbor inserting obstacle neighbor
func (a *Agent) InsertObstacleNeighbor(obstacle *Obstacle, rangeSq float32) {
	nextObstacle := obstacle.NextObstacle

	distSq := DistSqPointLineSegment(obstacle.Point, nextObstacle.Point, a.Position)

	if distSq < rangeSq {
		a.ObstacleNeighbors = append(a.ObstacleNeighbors, &ObstacleNeighbor{DistSq: distSq, Obstacle: obstacle})

		i := len(a.ObstacleNeighbors) - 1

		for {
			if i != 0 && distSq < a.ObstacleNeighbors[i-1].DistSq {
				a.ObstacleNeighbors[i] = a.ObstacleNeighbors[i-1]
				i--
			} else {
				break
			}
		}

		a.ObstacleNeighbors[i] = &ObstacleNeighbor{
			DistSq:   distSq,
			Obstacle: obstacle,
		}
	}
}

// Update agents velocity and position
func (a *Agent) Update() {
	a.Velocity = a.NewVelocity
	a.Position = Add(a.Position, MulOne(a.Velocity, Sim.TimeStep))
}

// LinearProgram1
// prefVelocityを適用できないため新しく計算する
// orcaLinesのあるLine(衝突しているLine？)に対して処理をおこなう
// 速度を変更してそれまでのラインに影響がないかを確認
func (a *Agent) LinearProgram1(lines []*Line, lineNo int, radius float32, optVelocity *Vector2, directionOpt bool) bool {
	var dotProduct, discriminant float32
	// pointとdirectionの内積
	dotProduct = Mul(lines[lineNo].Point, lines[lineNo].Direction)
	// 内積の二乗＋maxSpeedの二乗-pointの二乗(判別式)
	discriminant = float32(math.Pow(float64(dotProduct), 2)) + float32(math.Pow(float64(radius), 2)) - Sqr(lines[lineNo].Point)

	if discriminant < 0 {
		/* Max speed circle fully invalidates line lineNo. */
		return false
	}

	var sqrtDiscriminant, tLeft, tRight float32
	sqrtDiscriminant = float32(math.Sqrt(float64(discriminant)))
	tLeft = -dotProduct - sqrtDiscriminant
	tRight = -dotProduct + sqrtDiscriminant

	for i := 0; i < lineNo; i++ {
		var denominator, numerator float32
		// Det: スカラー値
		denominator = Det(lines[lineNo].Direction, lines[i].Direction)
		numerator = Det(lines[i].Direction, Sub(lines[lineNo].Point, lines[i].Point))

		if float32(math.Abs(float64(denominator))) <= RvoEpsilon {
			/* Lines lineNo and i are (almost) parallel. */
			if numerator < 0 {
				return false
			} else {
				continue
			}
		}

		var t float32
		t = numerator / denominator
		if denominator >= 0 {
			// 行iはlineNoより右側にある
			/* Line i bounds line lineNo on the right. */
			tRight = float32(math.Min(float64(tRight), float64(t)))
		} else {
			// 行iはlineNoより左側にある
			/* Line i bounds line lineNo on the left. */
			tLeft = float32(math.Max(float64(tLeft), float64(t)))
		}

		if tLeft > tRight {
			return false
		}
	}

	if directionOpt {
		/* Optimize direction. */
		if Mul(optVelocity, lines[lineNo].Direction) > 0 {
			//orcaLineに沿った方向の速度を設定する
			/* Take right extreme. */
			a.NewVelocity = Add(lines[lineNo].Point, MulOne(lines[lineNo].Direction, tRight))

		} else {
			// orcaLineに沿った方向の速度を設定する
			/* Take left extreme. */
			a.NewVelocity = Add(lines[lineNo].Point, MulOne(lines[lineNo].Direction, tLeft))

		}
	} else {
		/* Optimize closest point. */
		// t, tLeft, tRightは定数。orcaLineに沿った方向の速度を設定する
		t := Mul(lines[lineNo].Direction, Sub(optVelocity, lines[lineNo].Point))

		if t < tLeft {
			a.NewVelocity = Add(lines[lineNo].Point, MulOne(lines[lineNo].Direction, tLeft))

		} else if t > tRight {
			a.NewVelocity = Add(lines[lineNo].Point, MulOne(lines[lineNo].Direction, tRight))

		} else {
			a.NewVelocity = Add(lines[lineNo].Point, MulOne(lines[lineNo].Direction, t))

		}
	}

	return true
}

// LinearProgram2
// 速度変更
// linearprogramで正しい速度を算出できてないからObstacle3にいく？
func (a *Agent) LinearProgram2(lines []*Line, radius float32, optVelocity *Vector2, directionOpt bool) int {
	// 速度(a.NewVelocity)の前処理
	if directionOpt {
		// LP3から呼ばれた時のみ
		// 最適な方向と速度が決まっている場合：prefVelocityとmaxSpeedをかける
		/*
		 * Optimize direction. Note that the optimization velocity is of unit
		 * length in this case.
		 */
		a.NewVelocity = MulOne(optVelocity, radius)

	} else if Sqr(optVelocity) > float32(math.Pow(float64(radius), 2)) {
		// prefVelocityの距離が半径より大きいとき
		// 半径を越すとき、normarizeする
		/* Optimize closest point and outside circle. */
		a.NewVelocity = MulOne(Normalize(optVelocity), radius)

	} else {
		// prefVelocityの距離が半径以下のとき
		/* Optimize closest point and inside circle. */
		a.NewVelocity = optVelocity

	}

	// 全てのラインでtrueが帰れば終了
	for i := 0; i < len(lines); i++ {
		// 射影ベクトルのおおきさ？
		// 二週目は更新されたa.NewVelocityで再度ループ
		// NewVelocityで進むとあるラインとぶつかってしまう時
		if Det(lines[i].Direction, Sub(lines[i].Point, a.NewVelocity)) > 0 {
			// a.NewVelocityとlines[i]が安全ではないとき
			/* a.NewVelocity does not satisfy constraint i. Compute new optimal a.NewVelocity. */
			var tempResult *Vector2
			tempResult = a.NewVelocity

			// 速度を変更してそれまでのラインに影響がないかを確認
			if a.LinearProgram1(lines, i, radius, optVelocity, directionOpt) == false {
				// NewVelocityを元に戻す
				a.NewVelocity = tempResult
				// linearProgram3へ
				return i
			}
		}
	}
	return len(lines)
}

// LinearProgram3
// direction optimize
// 角度変更
func (a *Agent) LinearProgram3(lines []*Line, numObstLines int, beginLine int, radius float32) {
	var distance float32
	distance = 0.0
	for i := beginLine; i < len(lines); i++ {
		if Det(lines[i].Direction, Sub(lines[i].Point, a.NewVelocity)) > distance {
			/* Result does not satisfy constraint of line i. */
			var projLines []*Line
			projLines = make([]*Line, 0)
			for i := 0; i < numObstLines; i++ {
				projLines = append(projLines, lines[i])
			}

			for j := numObstLines; j < i; j++ {
				var line Line
				var determinant float32
				determinant = Det(lines[i].Direction, lines[j].Direction)

				if float32(math.Abs(float64(determinant))) <= RvoEpsilon {
					/* Line i and line j are parallel. */
					if Mul(lines[i].Direction, lines[j].Direction) > 0 {
						/* Line i and line j point in the same direction. */
						continue
					} else {
						/* Line i and line j point in opposite direction. */
						line.Point = MulOne(Add(lines[i].Point, lines[j].Point), 0.5)
					}
				} else {
					line.Point = Add(lines[i].Point, MulOne(lines[i].Direction, Det(lines[j].Direction, Sub(lines[i].Point, lines[j].Point))/determinant))
				}

				line.Direction = Normalize(Sub(lines[j].Direction, lines[i].Direction))

				projLines = append(projLines, &line)
			}

			var tempResult *Vector2
			tempResult = a.NewVelocity

			if a.LinearProgram2(projLines, radius, NewVector2(-lines[i].Direction.Y, lines[i].Direction.X), true) < len(projLines) {
				// ここは原則起こらないはず
				/* This should in principle not happen.  The a.NewVelocity is by definition
				 * already in the feasible region of this linear program. If it fails,
				 * it is due to small floating point error, and the current a.NewVelocity is
				 * kept.
				 */

				a.NewVelocity = tempResult

			}

			distance = Det(lines[i].Direction, Sub(lines[i].Point, a.NewVelocity))
		}
	}
}
