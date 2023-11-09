using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;


[Flags]
public enum StateOfWall
{
    LEFT = 1, 
    RIGHT = 2,
    UP = 4, 
    BOTTOM = 8,

    ISVISITED = 128,
}
public struct Position
{
    public int X;
    public int Y;
}

public struct Neighbour
{
    public Position Position;
    public StateOfWall SharedWall;
}

public static class MazeGenerator
{

    private static StateOfWall GetOppositeWall(StateOfWall wall)
    {
        switch (wall)
        {
            case StateOfWall.RIGHT: return StateOfWall.LEFT;
            case StateOfWall.LEFT: return StateOfWall.RIGHT;
            case StateOfWall.UP: return StateOfWall.BOTTOM;
            case StateOfWall.BOTTOM: return StateOfWall.UP;
            default: return StateOfWall.LEFT;
        }
    }

    private static StateOfWall[,] ApplyRecursiveBacktracker(StateOfWall[,] maze, int width, int height)
    {
        var rng = new System.Random();
        var positionStack = new Stack<Position>();
        var position = new Position { X = rng.Next(0, width), Y = rng.Next(0, height) };

        maze[position.X, position.Y] |= StateOfWall.ISVISITED;
        positionStack.Push(position);

        while (positionStack.Count > 0)
        {
            var current = positionStack.Pop();
            var neighbours = GetUnvisitedNeighbours(current, maze, width, height);

            if (neighbours.Count > 0)
            {
                positionStack.Push(current);

                var randIndex = rng.Next(0, neighbours.Count);
                var randomNeighbour = neighbours[randIndex];

                var nPosition = randomNeighbour.Position;
                maze[current.X, current.Y] &= ~randomNeighbour.SharedWall;
                maze[nPosition.X, nPosition.Y] &= ~GetOppositeWall(randomNeighbour.SharedWall);
                maze[nPosition.X, nPosition.Y] |= StateOfWall.ISVISITED;

                positionStack.Push(nPosition);
            }
        }
        return maze;
    }

    private static List<Neighbour> GetUnvisitedNeighbours(Position p, StateOfWall[,] maze, int width, int height)
    {
        var list = new List<Neighbour>();

        if (p.X > 0)
        {
            if (!maze[p.X - 1, p.Y].HasFlag(StateOfWall.ISVISITED))
            {
                list.Add(new Neighbour
                {
                    Position = new Position
                    {
                        X = p.X - 1,
                        Y = p.Y
                    },
                    SharedWall = StateOfWall.LEFT
                });
            }
        }

        if (p.Y > 0)
        {
            if (!maze[p.X, p.Y - 1].HasFlag(StateOfWall.ISVISITED))
            {
                list.Add(new Neighbour
                {
                    Position = new Position
                    {
                        X = p.X,
                        Y = p.Y - 1
                    },
                    SharedWall = StateOfWall.BOTTOM
                });
            }
        }

        if (p.Y < height - 1)
        {
            if (!maze[p.X, p.Y + 1].HasFlag(StateOfWall.ISVISITED))
            {
                list.Add(new Neighbour
                {
                    Position = new Position
                    {
                        X = p.X,
                        Y = p.Y + 1
                    },
                    SharedWall = StateOfWall.UP
                });
            }
        }

        if (p.X < width - 1)
        {
            if (!maze[p.X + 1, p.Y].HasFlag(StateOfWall.ISVISITED))
            {
                list.Add(new Neighbour
                {
                    Position = new Position
                    {
                        X = p.X + 1,
                        Y = p.Y
                    },
                    SharedWall = StateOfWall.RIGHT
                });
            }
        }
        Debug.Log(list.Count);
        return list;
    }

    public static StateOfWall[,] Generate(int width, int height)
    {
        StateOfWall[,] maze = new StateOfWall[width, height];
        StateOfWall initial = StateOfWall.RIGHT | StateOfWall.LEFT | StateOfWall.UP | StateOfWall.BOTTOM;
        for (int i = 0; i < width; ++i)
        {
            for (int j = 0; j < height; ++j)
            {
                maze[i, j] = initial;
            }
        }
        maze[0, UnityEngine.Random.Range(0, height)] &= ~StateOfWall.LEFT;
        maze[width-1, UnityEngine.Random.Range(0, height)] &= ~StateOfWall.RIGHT; 
        return ApplyRecursiveBacktracker(maze, width, height);
    }

    public static List<Position> FindPath(Position start, Position goal, StateOfWall[,] maze, int width, int height)
    {
        List<Position> path = new List<Position>();
        Dictionary<Position, Position> cameFrom = new Dictionary<Position, Position>();
        Dictionary<Position, float> gScore = new Dictionary<Position, float>();
        Dictionary<Position, float> fScore = new Dictionary<Position, float>();

        List<Position> openSet = new List<Position>();
        openSet.Add(start);
        List<Position> closedSet = new List<Position>();

        gScore[start] = 0;
        fScore[start] = HeuristicCostEstimate(start, goal);

        while (openSet.Count > 0)
        {
            Position current = GetLowestFScore(openSet, fScore);

            if (current.X == goal.X && current.Y == goal.Y)
            {
                Debug.Log("Found path");
                path = ReconstructPath(cameFrom, current);
                return path;
            }
            closedSet.Add(current);

            List<Neighbour> neighbours = GetCurrentNeighbours(current, maze, width, height);
    
            foreach (var neighbor in neighbours)
            {
                float tentativeGScore = gScore[current] + 1;

                if (!gScore.ContainsKey(neighbor.Position) || tentativeGScore < gScore[neighbor.Position])
                {
                    cameFrom[neighbor.Position] = current;
                    gScore[neighbor.Position] = tentativeGScore;
                    fScore[neighbor.Position] = gScore[neighbor.Position] + HeuristicCostEstimate(neighbor.Position, goal);

                    if (!openSet.Contains(neighbor.Position))
                    {
                        openSet.Add(neighbor.Position);
                    }
                }
            }
            foreach (var positionToRemove in closedSet)
            {
                openSet.Remove(positionToRemove);
            }

            closedSet.Clear();
        }
        return path;
    }

    private static float HeuristicCostEstimate(Position start, Position goal)
    {
        return Mathf.Abs(start.X - goal.X) + Mathf.Abs(start.Y - goal.Y);
    }

    private static Position GetLowestFScore(List<Position> openSet, Dictionary<Position, float> fScore)
    {
        float minFScore = float.MaxValue;
        Position minPosition = openSet[0];

        foreach (var position in openSet)
        {
            if (fScore.ContainsKey(position) && fScore[position] < minFScore)
            {
                minFScore = fScore[position];
                minPosition = position;
            }
        }

        return minPosition;
    }

    private static List<Position> ReconstructPath(Dictionary<Position, Position> cameFrom, Position current)
    {
        List<Position> path = new List<Position>();

        while (cameFrom.ContainsKey(current))
        {
            path.Add(current);
            current = cameFrom[current];
        }

        path.Reverse();
        return path;
    }

    private static List<Neighbour> GetCurrentNeighbours(Position p, StateOfWall[,] maze, int width, int height)
    {
        var list = new List<Neighbour>();

        if (p.X > 0)
        {
            list.Add(new Neighbour
            {
                Position = new Position
                {
                    X = p.X - 1,
                    Y = p.Y
                },
                SharedWall = StateOfWall.LEFT
            });
        }

        if (p.Y > 0)
        {
            list.Add(new Neighbour
            {
                Position = new Position
                {
                    X = p.X,
                    Y = p.Y - 1
                },
                SharedWall = StateOfWall.BOTTOM
            });
        }

        if (p.Y < height - 1)
        {
            list.Add(new Neighbour
            {
                Position = new Position
                {
                    X = p.X,
                    Y = p.Y + 1
                },
                SharedWall = StateOfWall.UP
            });
        }

        if (p.X < width - 1)
        {
                list.Add(new Neighbour
                {
                    Position = new Position
                    {
                        X = p.X + 1,
                        Y = p.Y
                    },
                    SharedWall = StateOfWall.RIGHT
                });
        }
        Debug.Log(list.Count);
        return list;
    }
}