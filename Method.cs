using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using xna = Microsoft.Xna.Framework;
using URWPGSim2D.Common;
using URWPGSim2D.StrategyLoader;

namespace URWPGSim2D.Strategy
{
    class Method
    {
        #region 鱼所需转动的角度
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

        /// <summary>
        /// 返回转动方向 
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

        #region 鱼小空间内转弯速度推荐决策值（权衡速度与空间限制）
        public static int GetxzdVcode(float angvel)
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

        #region 根据距离推荐速度决策值
        /// <summary>
        /// 根据距离推荐速度决策值,推荐距离为300内时调用
        /// </summary>
        /// <param name="distance">相距距离</param>
        /// <returns>Vcode档位</returns>
        public static int GetVcodeByDitence(float distance)
        {
            if (distance < 300 && distance >= 150)
                return 8;
            else
                return 6;
        }
        #endregion

        #region 获得两点距离
        public static float GetDistance(xna.Vector3 a, xna.Vector3 b)
        {
            return (float)Math.Sqrt(Math.Pow(a.X - b.X, 2) + Math.Pow(a.Z - b.Z, 2));
        }

        #endregion

        #region 通过球与目标点，获得顶球点坐标
        /// <summary>
        /// 通过球与目标点，获得顶球点坐标
        /// </summary>
        /// <param name="ball">球对象</param>
        /// <param name="dest">目标位置</param>
        /// <returns>顶球点坐标</returns>
        public static xna.Vector3 getPoint(xna.Vector3 ball, xna.Vector3 dest)
        {
            xna.Vector3 aimvector = new xna.Vector3(dest.X - ball.X, 0, dest.Z - ball.Z);
            float aimrad = StrategyHelper.Helpers.GetAngleDegree(aimvector);
            xna.Vector3 point = new xna.Vector3();
            point.Z = ball.Z - 58 * (float)Math.Sin(aimrad);
            point.X = ball.X - 58 * (float)Math.Cos(aimrad);
            return point;
        }
        #endregion
    }
}
