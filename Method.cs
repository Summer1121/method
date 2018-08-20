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
        #region 返回鱼到定点的实时T速度档位决策值
        /// <summary>
        /// 实时返回角速度档位
        /// </summary>
        /// <param name="angvel">由Getxzdangle（）函数返回的角度值</param>
        /// <returns></returns>
        public static int TransfromAngletoTCode(float angvel)
        {
            if (angvel < 0)
                return 14;
            else if (angvel == 0)
                return 7;
            else
                return 0;
        }
        /// <summary>
        /// 返回鱼到达定点需要转的角度
        /// </summary>
        /// <param name="fish">机器鱼对象</param>
        /// <param name="aimPosition">目标点坐标</param>
        /// <returns></returns>
        public static float Getxzdangle(RoboFish fish, xna.Vector3 aimPosition)
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
    }
}
