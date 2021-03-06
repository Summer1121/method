﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using xna = Microsoft.Xna.Framework;
using URWPGSim2D.Common;
using URWPGSim2D.StrategyLoader;
using URWPGSim2D.Core;

namespace URWPGSim2D.Strategy
{
    class Method
    {
        //15个档位的实际速度
        private static float[] Vtable = { 0, 9.0152f, 31.5533f, 60.4020f, 88.3492f, 110.6591f, 132.7829f, 152.1562f, 172.9048f, 204.6457f, 268.5165f, 289.3336f, 295.6592f, 293.9903f, 303.6920f };
        private static float[] Ttable = { -0.3552f, -0.2921f, -0.2200f, -0.1731f, -0.1235f, -0.0784f, -0.0469f, 0, 0.0469f, 0.0784f, 0.1235f, 0.1731f, 0.2200f, 0.2921f, 0.3438f };
        public static float getVtable(int t)
        {
            return Vtable[t];
        }
        public static float getTtable(int t)
        {
            return Ttable[t];
        }
        #region 鱼体简单快速运动封装 控制鱼到达某点（最常用运动函数）
        /// <summary>
        /// 鱼体简单快速运动封装
        /// </summary>
        /// <param name="decision">鱼的决策对象</param>
        /// <param name="fish">鱼的参数只读对象</param>
        /// <param name="aim_point">目标位置</param>
        /// <param name="Vcode1">巡航速度最大值,可以取14，实际档位由程序决定</param>
        /// <param name="Vcode2">减速第一阶段，默认为8</param>
        /// <param name="Vcode3">减速第二阶段，默认为6</param>
        static public void approachToPoint(ref Decision decision, RoboFish fish, xna.Vector3 aim_point, int Vcode1, int Vcode2, int Vcode3)
        {
            float angle = Getxzdangle(fish, aim_point);
            int Tcode = GetxzdTcode(angle);
            decision.TCode = Tcode;
            float distance = GetDistance(fish.PolygonVertices[0], aim_point);
            if (distance > 200)
            {
                int autoVcode = GetxzdVcode(fish, aim_point);
                decision.VCode = Vcode1 < autoVcode ? Vcode1 : autoVcode;//较大距离时巡航速度
            }
            else if (distance >= 100)
            {
                decision.VCode = Vcode2;//第一阶段减速
            }
            else
            {
                decision.VCode = Vcode3;//第二阶段减速
            }
        }
        #endregion

        #region 鱼所需转动的角度
        /// <summary>
        /// 返回所需转过的角度
        /// </summary>
        /// <param name="fish"></param>
        /// <param name="aimPosition"></param>
        /// <returns>返回所需转过的角度 </returns>
        public static float Getxzdangle(RoboFish fish, xna.Vector3 aimPosition)
        {
            xna.Vector3 aimVector;
            aimVector.X = aimPosition.X - fish.PolygonVertices[0].X;
            aimVector.Z = aimPosition.Z - fish.PolygonVertices[0].Z;
            aimVector.Y = 0;
            //float aimAngle = StrategyHelper.Helpers.GetAngleDegree(aimVector);
            xna.Vector3 fishRad = new xna.Vector3((float)Math.Cos(fish.BodyDirectionRad), 0, (float)Math.Sin(fish.BodyDirectionRad));
            //公式：θ=atan2(v2.y,v2.x)−atan2(v1.y,v1.x)
            //atan2的取值范围是[−π,π]，在进行相减之后得到的夹角是在[−2π,2π]，
            //因此当得到的结果大于π时，对结果减去2π，当结果小于−π时，对结果加上2π
            //虽然与一般坐标方向不一致，但是象限都是3 4 1 2的顺序，所以仍然成立
            //但是仍需验证
            float theta = (float)Math.Atan2(aimVector.Z, aimVector.X) - (float)Math.Atan2(fishRad.Z, fishRad.X);
            if (theta > Math.PI)
                theta -= (float)(2 * Math.PI);
            else if (theta < -Math.PI)
                theta += (float)(2 * Math.PI);
            return theta;
        }

        /// <summary>
        /// 返回鱼到达定点需要转的角度
        /// </summary>
        /// <param name="fish">机器鱼对象</param>
        /// <param name="aimPosition">目标点坐标</param>
        /// <returns>鱼到达定点需要转的角度</returns>
        public static float _Getxzdangle(RoboFish fish, xna.Vector3 aimPosition)
        {
            float cur_x = fish.PositionMm.X;
            float cur_z = fish.PositionMm.Z;
            float dest_x = aimPosition.X;
            float dest_z = aimPosition.Z;
            float fish_rad = fish.BodyDirectionRad;
            //鱼体方 向mission.TeamsRef[teamId].Fishes[i].BodyDirectionRad
            float curangle;
            float xzdangle = fish_rad;
            curangle = (float)(Math.Abs(Math.Atan((cur_x - dest_x) / (cur_z - dest_z))));
            if ((cur_x > dest_x) && (cur_z > dest_z))
            {//以球为中心，当鱼在球的右下方
                if (fish_rad < 0 && fish_rad > -Math.PI / 2)
                {
                    xzdangle = -(float)(Math.PI / 2 + curangle + fish_rad);
                }
                else if (fish_rad > (-Math.PI) && fish_rad < -(Math.PI / 2))
                {
                    xzdangle = (float)(-Math.PI / 2 - fish_rad - curangle);
                }

                else if (1.5 * Math.PI - fish_rad - curangle < fish_rad + 0.5 * Math.PI)
                    xzdangle = (float)(Math.PI * 1.5 - fish_rad - curangle);
                else
                    xzdangle = (float)(fish_rad + 0.5 * Math.PI);
            }
            else if ((cur_x > dest_x) && (cur_z < dest_z))
            {//以球为中心，当鱼在球的右上方
                if (fish_rad < ((Math.PI / 2 + curangle)) && (-(Math.PI / 2 - curangle)) < fish_rad)
                {
                    xzdangle = (float)(Math.PI / 2 + curangle - fish_rad);
                }
                else if ((-(Math.PI / 2 - curangle) > fish_rad) && fish_rad > -(Math.PI))
                {
                    xzdangle = (float)(Math.PI * 2 + fish_rad - curangle);
                    xzdangle = -xzdangle;
                }
                else if (fish_rad > ((Math.PI / 2 + curangle)) && fish_rad < (Math.PI))
                {
                    xzdangle = (float)(fish_rad - Math.PI / 2 - curangle);
                    xzdangle = -xzdangle;
                }
            }
            else if ((cur_x < dest_x) && (cur_z < dest_z))
            {//以球为中心，当鱼在球的左上方
                if (fish_rad >= 0 && fish_rad < Math.PI)
                {
                    xzdangle = (float)(curangle - fish_rad);

                }
                else if (fish_rad > 0.5 * Math.PI && fish_rad < Math.PI)
                {
                    xzdangle = (float)(fish_rad - curangle);

                }
                else
                {
                    if (-fish_rad + curangle > 2 * Math.PI + fish_rad - curangle)
                        xzdangle = -(float)(2 * Math.PI + fish_rad - curangle);
                    else
                        xzdangle = (float)(-fish_rad + curangle);
                }

            }
            else if ((cur_x < dest_x) && (cur_z > dest_z))
            {//以球为中心，当鱼在球的左下方
                if (fish_rad >= 0 && fish_rad <= Math.PI)
                {
                    if (curangle + fish_rad < Math.PI * 2 - curangle - fish_rad)
                        xzdangle = -(float)(curangle + fish_rad);
                    else
                        xzdangle = (float)(Math.PI * 2 - curangle - fish_rad);
                }

                else
                {
                    if (fish_rad > -Math.PI && fish_rad < -0.5 * Math.PI)
                        xzdangle = (float)-(fish_rad + curangle);
                    else
                        xzdangle = (float)(fish_rad + curangle);
                }
            }
            return xzdangle;
        }


        #endregion

        //#region dribble算法封装
        ///// <summary>
        ///// dribble算法封装
        ///// </summary>
        ///// <param name="decision">每周期机器鱼执行策略，包含速度档位，转弯档位。</param>
        ///// <param name="fish">目标仿真机器鱼参数，包含当前位置、速度信息。</param>
        ///// <param name="destPtMm">目标点。</param>
        ///// <param name="destDirRad">目标方向。</param>
        ///// <param name="angleTheta1">鱼体方向与目标方向角度差的阈值一。
        /////   角度差在此阈值范围内，则赋给机器鱼一个合理的速度档位（见参数disThreshold说明）。</param>
        ///// <param name="angleTheta2">鱼体方向与目标方向角度差的阈值二。角度差小于此阈值，则机器鱼直线游动；
        ///// 角度差大于此阈值，则机器鱼调整游动方向。</param>
        ///// <param name="disThreshold">距离阈值。距离大于此阈值，机器鱼以速度档位VCode1游动；
        ///// 距离小于此阈值，机器鱼以速度档位VCode2游动。</param>
        ///// /// <param name="VCode1">直游档位1（默认6档）。</param>
        ///// /// <param name="VCode2">直游档位2（默认4档）。</param>
        ///// <param name="cycles">速度和转弯档位之间切换所需周期数经验值。建议取值范围在5-20之间。此参数作用是防止机器鱼“转过”,建议取5。</param>
        ///// <param name="msPerCycle">每个仿真周期的毫秒数，传递固定参数，不能修改。</param>
        ///// <param name="flag">机器鱼坐标选择标准，true为PositionMm，即鱼体绘图中心；false为PolygonVertices[0]，即鱼头点。</param>
        ///// <param name="dest">目标终点</param>
        ///// <returns></returns>
        //public static bool dribble(ref Decision decision, RoboFish fish, xna.Vector3 ball, float destDirRad,
        //    float angleTheta1, float angleTheta2, float disThreshold, int VCode1, int VCode2, int cycles, int msPerCycle, bool flag, 
        //    xna.Vector3 dest_point)
        //{
        //    xna.Vector3 aim_vector = new xna.Vector3(dest_point.X - ball.X, 0, dest_point.Z - ball.Z);
        //    float aim_rad = GetRadByVector(aim_vector);
        //    if (GetDistance(fish.PolygonVertices[0], ball) <= 60)//判断是否在顶球范围内
        //    {
        //        StrategyHelper.Helpers.Dribble(ref decision, fish, ball, aim_rad, angleTheta1, angleTheta2, disThreshold, VCode1, VCode2, cycles, msPerCycle, true);
        //    }
        //    else
        //    {
        //        if (flag0 == 3)
        //        {
        //            xna.Vector3 dest_point = new xna.Vector3(-600, 0, 200);//顶球终点
        //            xna.Vector3 aim_position = Method.getPoint(balls[0], dest_point, 58);//预顶球点
        //            xna.Vector3 aim_vector = new xna.Vector3(dest_point.X - balls[0].X, 0, dest_point.Z - balls[0].Z);
        //            float aim_rad = Method.GetRadByVector(aim_vector);//在顶球点的目标位姿
        //                                                              //StrategyHelper.Helpers.PoseToPose(ref decisions[0], fishs[0], aim_position, aim_rad, 30f, 0f, mission.CommonPara.MsPerCycle, ref times[0], true);
        //            Method.approachToPoint(ref decisions[0], fishs[0], aim_position, 14, 8, 6);
        //            if (Method.ifOnVector(fishs[0].PolygonVertices[0], balls[0], dest_point))
        //            {
        //                times[0] = 0;
        //                flag0 = 4;
        //            }
        //        }
        //        if (flag0 == 4)
        //        {
        //            xna.Vector3 dest_point = new xna.Vector3(-600, 0, 200);//顶球终点
        //            xna.Vector3 aim_position = Method.getPoint(balls[0], dest_point, 58);//顶球点
        //                                                                                 ////Method.approachToPoint(ref decisions[0], fishs[0], aim_position, 6, 5, 4);
        //            xna.Vector3 aim_vector = new xna.Vector3(dest_point.X - balls[0].X, 0, dest_point.Z - balls[0].Z);
        //            float aim_rad = Method.GetRadByVector(aim_vector);//在顶球点的目标位姿
        //                                                              //Method.turn(ref decisions[0], fishs[0], aim_rad);
        //            StrategyHelper.Helpers.PoseToPose(ref decisions[0], fishs[0], aim_position, aim_rad, 5f, 0f, mission.CommonPara.MsPerCycle, ref times[0]);


        //            if (Math.Abs(fishs[0].BodyDirectionRad - aim_rad) < 0.05 * Math.PI)
        //            {
        //                times[0] = 0;
        //                flag0 = 5;
        //            }
        //        }
        //}
        //#endregion

        #region 顶球终点为一条线的算法 获得线段上某点作为当前顶球终点
        /// <summary>
        /// 获得线段上某点作为当前顶球终点
        /// </summary>
        /// <param name="fish">机器鱼只读属性对象</param>
        /// <param name="ball">球的坐标</param>
        /// <param name="point1">线段一端点坐标</param>
        /// <param name="point2">线段另一端点坐标</param>
        /// <returns>返回当前鱼、球延长线与线段上交点  如不在线段上，则返回线段中点</returns>
        public static xna.Vector3 getDest(RoboFish fish, xna.Vector3 ball, xna.Vector3 point1, xna.Vector3 point2)
        {
            float k1 = (ball.Z - fish.PositionMm.X) / (ball.X - fish.PositionMm.X);
            float b1 = ball.Z - k1 * ball.X;

            float k2 = (point1.Z - point2.Z) / (point1.X - point2.X);
            float b2 = point1.Z - k2 * point1.X;

            xna.Vector3 res = new xna.Vector3();
            res.X = (b2 - b1) / (k1 - k2);
            res.Z = (k1 * b2 - k2 * b1) / (k1 - k2);
            res.Y = 0;
            if ((res.X > point1.X && res.X < point2.X) || (res.X < point1.X && res.X > point2.X) || (res.Z > point1.Z && res.Z < point2.Z) || (res.Z < point1.Z && res.Z > point2.Z))
                return res;
            else
            {
                res.X = (point1.X + point2.X) / 2;
                res.Z = (point1.Z + point2.Z) / 2;
                return res;
            }
        }

        #endregion

        #region 在当前点做小范围内转弯
        public static void turn(ref Decision decision, RoboFish fish, float aim_direction)
        {
            float angle = aim_direction - fish.BodyDirectionRad;
            if (angle < -2 * Math.PI) angle += (float)(2 * Math.PI);
            else if (angle > 2 * Math.PI) angle -= (float)(2 * Math.PI);
            decision.TCode = GetxzdTcode(angle);
            decision.VCode = 1;
        }

        #endregion

        #region 鱼到定点的推荐T速度档位决策值
        /// <summary>
        /// 实时返回角速度档位
        /// </summary>
        /// <param name="angvel">由Getxzdangle（）函数返回的角度值</param>
        /// <returns>返回角速度档位</returns>
        public static int GetxzdTcode(float angvel)
        {
            //float interval = 1f / 7;//每个划分区间的宽度
            //if (angvel == 0)
            //    return 7;

            //else if (angvel < interval * Math.PI && angvel > 0)
            //    return 8;
            //else if (angvel < 2 * interval * Math.PI && angvel >= interval * Math.PI)
            //    return 9;
            //else if (angvel < 3 * interval * Math.PI && angvel >= 2 * interval * Math.PI)
            //    return 10;
            //else if (angvel < 4 * interval * Math.PI && angvel >= 3 * interval * Math.PI)
            //    return 11;
            //else if (angvel < 5 * interval * Math.PI && angvel >= 4 * interval * Math.PI)
            //    return 12;
            //else if (angvel < 6 * interval * Math.PI && angvel >= 5 * interval * Math.PI)
            //    return 13;
            //else if (angvel <= Math.PI && angvel >= 5 * interval * Math.PI)
            //    return 14;


            //else if (-angvel < interval * Math.PI && -angvel > 0)
            //    return 6;
            //else if (-angvel < 2 * interval * Math.PI && -angvel >= interval * Math.PI)
            //    return 5;
            //else if (-angvel < 3 * interval * Math.PI && -angvel >= 2 * interval * Math.PI)
            //    return 4;
            //else if (-angvel < 4 * interval * Math.PI && -angvel >= 3 * interval * Math.PI)
            //    return 3;
            //else if (-angvel < 5 * interval * Math.PI && -angvel >= 4 * interval * Math.PI)
            //    return 2;
            //else if (-angvel < 6 * interval * Math.PI && -angvel >= 5 * interval * Math.PI)
            //    return 1;
            //else if (-angvel <= Math.PI && -angvel >= 5 * interval * Math.PI)
            //    return 0;

            //else return 7;


            if (angvel < 0) return 0;
            else if (angvel > 0) return 14;
            else return 7;

        }
        #endregion

        #region 鱼转弯速度推荐决策值


        //完整控制版的简略版，仅仅返回速度档位
        public static int GetxzdVcode(RoboFish fish, xna.Vector3 destpoint)
        {
            float rad = Getxzdangle(fish, destpoint);
            float t = Math.Abs(rad / getTtable(0));//最大转弯幅度预计时间
            float dis = GetDistance(fish.PolygonVertices[0], destpoint);
            for (int i = 0; i <= 14; i++)
            {
                if (dis / t <= getVtable(i))
                {
                    return i;
                }
            }
            return 14;
        }
        public static int _GetxzdVcode(float angvel)
        {
            float interval = 0.2f;//每个划分区间的宽度
            if (angvel <= interval * Math.PI && angvel >= -interval * Math.PI)
                return 14;
            else if (angvel <= 2 * interval * Math.PI && angvel >= -2 * interval * Math.PI)
                return 11;
            else if (angvel <= 3 * interval * Math.PI && angvel >= -3 * interval * Math.PI)
                return 8;
            else if (angvel <= 4 * interval * Math.PI && angvel >= -4 * interval * Math.PI)
                return 5;
            else return 2;

        }
        #endregion

        #region 获得两点距离
        public static float GetDistance(xna.Vector3 a, xna.Vector3 b)
        {
            return (float)Math.Sqrt(Math.Pow(a.X - b.X, 2) + Math.Pow(a.Z - b.Z, 2));
        }

        #endregion

        #region 通过球与目标点，获得 鱼头 顶球点坐标
        /// <summary>
        /// 通过球与目标点，获得顶球点坐标
        /// </summary>
        /// <param name="ball">球对象</param>
        /// <param name="dest">目标位置</param>
        /// <param name="r">球的半径，一般球的半径为58，部分项目有差异</param>
        /// <returns>顶球点坐标</returns>
        public static xna.Vector3 getPoint(xna.Vector3 ball, xna.Vector3 dest, int r)
        {
            xna.Vector3 aimvector = new xna.Vector3(dest.X - ball.X, 0, dest.Z - ball.Z);
            float aimrad = GetRadByVector(aimvector);
            xna.Vector3 point = new xna.Vector3();
            point.X = ball.X - (float)(r * Math.Cos(aimrad));
            point.Z = ball.Z - (float)(r * Math.Sin(aimrad));
            point.Y = 0;
            return point;
        }
        public static xna.Vector3 GetPointOfStart(xna.Vector3 ball, xna.Vector3 dest)
        {
            xna.Vector3 point = new xna.Vector3(0, 0, 0);

            if (ball.X > dest.X && ball.Z > dest.Z)//球在洞的右下角
            {
                float degree = StrategyHelper.Helpers.GetAngleDegree(dest - ball);
                float rad = xna.MathHelper.ToRadians(degree);
                rad = (float)Math.PI + rad;
                point = new xna.Vector3(ball.X + 58 * (float)Math.Cos(rad), 0, ball.Z + 58 * (float)Math.Sin(rad));
            }
            if (ball.X > dest.X && ball.Z < dest.Z)//秋在洞的右上角
            {
                float degree = StrategyHelper.Helpers.GetAngleDegree(dest - ball);
                float rad = xna.MathHelper.ToRadians(degree);
                rad = (float)Math.PI - rad;
                point = new xna.Vector3(ball.X + 58 * (float)Math.Cos(rad), 0, ball.Z - 58 * (float)Math.Sin(rad));
            }
            if (ball.X < dest.X && ball.Z > dest.Z)//秋在洞的左下角
            {
                float degree = StrategyHelper.Helpers.GetAngleDegree(dest - ball);
                float rad = xna.MathHelper.ToRadians(degree);
                rad = -rad;
                point = new xna.Vector3(ball.X - 58 * (float)Math.Cos(rad), 0, ball.Z + 58 * (float)Math.Sin(rad));
            }
            if (ball.X < dest.X && ball.Z < dest.Z)//秋在洞的左上角
            {
                float degree = StrategyHelper.Helpers.GetAngleDegree(dest - ball);
                float rad = xna.MathHelper.ToRadians(degree);
                point = new xna.Vector3(ball.X - 58 * (float)Math.Cos(rad), 0, ball.Z - 58 * (float)Math.Sin(rad));
            }

            return point;
        }
        #endregion

        #region 判断某点是否在某条向量的反向延长线上（附近）
        /// <summary>
        /// 判断某点是否在某条向量或者延长线上（附近）
        /// </summary>
        /// <param name="curr">所要判断的点的坐标</param>
        /// <param name="start">向量起始点</param>
        /// <param name="end">向量指向点</param>
        /// <returns>是或者不是</returns>
        public static bool ifOnVector(xna.Vector3 curr, xna.Vector3 start, xna.Vector3 end)
        {
            float xtemp = start.X - ((start.Z - curr.Z) * (end.X - start.X) / (end.Z - start.Z));
            if (end.Z > curr.Z && start.Z < curr.Z) return false;
            if (end.Z < curr.Z && start.Z > curr.Z) return false;
            if (start.Z == end.Z)
            {
                if (end.X > curr.X && start.X < curr.X) return false;
                if (end.X < curr.X && start.X > curr.X) return false;
            }

            if (curr.X + 50 >= xtemp && curr.X - 50 <= xtemp)
                return true;
            else return false;
        }
        #endregion

        #region 顶球点和鱼的位置冲突，无法直线到达，防止无限顶球
        /// <summary>
        /// 顶球点和鱼的位置冲突，无法直线到达，防止无限顶球
        /// </summary>
        /// <param name="decision">决策变量</param>
        /// <param name="fish">鱼的只读属性</param>
        /// <param name="ball">球的坐标</param>
        /// <param name="point">顶球点</param>
        /// <returns>是否进行了调整</returns>
        public static bool go_aside(ref Decision decision, RoboFish fish, xna.Vector3 ball, xna.Vector3 point)
        {
            //获取顶球点的对称点
            xna.Vector3 point2 = new xna.Vector3(ball.X + (ball.X - point.X), 0, ball.Z + (ball.Z - point.Z));
            if (GetDistance(fish.PolygonVertices[0], point2) < 70 && GetDistance(fish.PolygonVertices[0], ball) < 80)
            {
                xna.Vector3 vector_point = new xna.Vector3(point.X - ball.X, 0, point.Z - ball.Z);//从球到顶球点的向量
                xna.Vector3 vector_fish = new xna.Vector3(fish.PolygonVertices[0].X - ball.X, 0, fish.PolygonVertices[0].Z - ball.Z);
                float rad_point = GetRadByVector(vector_point);
                float rad_fish = GetRadByVector(vector_fish);
                float angle = rad_point - rad_fish;
                if (angle > Math.PI) angle -= (float)(2 * Math.PI);
                else if (angle < -Math.PI) angle += (float)(2 * Math.PI);

                if (angle > 0)
                    decision.TCode = 0;
                else decision.TCode = 14;
                decision.VCode = 14;

                //判断鱼头位置而调整方向

                return true;//表示需要调整
            }
            return false;
        }
        #endregion

        #region 将向量换算为角度
        public static float GetRadByVector(xna.Vector3 vec)
        {
            float rad = (float)Math.Atan2(vec.Z, vec.X);
            return rad;
        }

        #endregion

        #region 在两点之间取若干点，分阶段，减小目标距离，增大夹角，以达到精确控制
        /// <summary>
        /// 返回目标向量上距离为300的点作为临时点
        /// </summary>
        /// <param name="start">起始点</param>
        /// <param name="end">终止点</param>
        /// <returns>返回临时点的坐标</returns>
        public static xna.Vector3 GetTempPoint(xna.Vector3 start, xna.Vector3 end)
        {
            xna.Vector3 temp = new xna.Vector3();
            xna.Vector3 vector = new xna.Vector3(end.X - start.X, 0, end.Z - start.Z);
            float rad = GetRadByVector(vector);
            temp.X = (float)(start.X + 300 * Math.Cos(rad));
            temp.Z = (float)(start.Z + 300 * Math.Sin(rad));
            temp.Y = 0;
            return temp;
        }
        #endregion

        #region 根据预计角速度、速度获得策略参数以控制鱼的运动(不考虑减速的运动)
        public static void approachToPoint2(ref Decision decision, RoboFish fish, xna.Vector3 destpoint)
        {
            float rad = Getxzdangle(fish, destpoint);
            decision.TCode = GetxzdTcode(rad);
            float t = Math.Abs(rad / getTtable(0));//最大转弯幅度预计时间
            float dis = GetDistance(fish.PolygonVertices[0], destpoint);
            for (int i = 0; i <= 14; i++)
            {
                if (dis / t <= getVtable(i))
                {
                    decision.VCode = i;
                    break;
                }
            }
        }
        #endregion

        #region 自制顶球算法
        /// <summary>
        /// 顶球算法
        /// </summary>
        /// <param name="decision">鱼的决策对象</param>
        /// <param name="fish">鱼的属性只读对象</param>
        /// <param name="ball">球的坐标</param>
        /// <param name="dest">目标坐标</param>
        /// <param name="angleThreshold">角度阈值，在此阈值内，进行顶球，超出则寻找顶球点</param>
        /// <param name="Vcode1">速度最大值（越小越准）</param>
        /// <param name="r">球的半径，大部分比赛默认58</param>
        /// <param name="mission">比赛仿真对象</param>
        /// <param name="times">标志变量</param>
        public static void Dribble(ref Decision decision, RoboFish fish, xna.Vector3 ball, xna.Vector3 dest, float angleThreshold, int Vcode1, int r, Mission mission, ref int times)
        {
            xna.Vector3 point = getPoint(ball, dest, 58);
            xna.Vector3 vector = new xna.Vector3(dest.X - ball.X, 0, dest.Z - ball.Z);
            float rad = GetRadByVector(vector);
            if (Math.Abs(fish.BodyDirectionRad - rad) < angleThreshold / 180 * Math.PI && GetDistance(fish.PolygonVertices[0], ball) <= 65)//在顶球区域内，进行顶球
            {
                approachToPoint(ref decision, fish, dest, Vcode1, 8, 6);
            }
            else //不在顶球区域内，寻找顶球点
            {
                StrategyHelper.Helpers.PoseToPose(ref decision, fish, point, rad, 2, 30, mission.CommonPara.MsPerCycle, ref times);
                //approachToPoint(ref decision, fish, point, 14, 4, 2);
            }
        }
        #endregion



    }
}
