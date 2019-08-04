using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace MINAV
{
    public class SolidSpanGroupCPlus
    {
        [DllImport("MiNavMeshPlus")]
        public static extern IntPtr CreateSolidSpanGroup(IntPtr voxSpace);

        [DllImport("MiNavMeshPlus")]
        public static extern IntPtr GetSolidSpanGrids(IntPtr solidSpanGroup);

        [DllImport("MiNavMeshPlus")]
        public static extern int GetGridCount(IntPtr solidSpanGroup);

    }
}
