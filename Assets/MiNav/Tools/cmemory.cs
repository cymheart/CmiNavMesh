using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Reflection.Emit;
using System.Runtime.InteropServices;
using System.Text;

namespace MINAV
{
    public static class cmemory
    {
        static Action<IntPtr, byte, int> memsetDelegate;
        static cmemory()
        {
            DynamicMethod dynamicMethod = new DynamicMethod(
                "memset",
                MethodAttributes.Public | MethodAttributes.Static, CallingConventions.Standard,
                null,
                new[] { typeof(IntPtr), typeof(byte), typeof(int) }, typeof(cmemory), true
                );
            ILGenerator generator = dynamicMethod.GetILGenerator();
            generator.Emit(OpCodes.Ldarg_0);
            generator.Emit(OpCodes.Ldarg_1);
            generator.Emit(OpCodes.Ldarg_2);
            generator.Emit(OpCodes.Initblk);
            generator.Emit(OpCodes.Ret);
            memsetDelegate = (Action<IntPtr, byte, int>)dynamicMethod.CreateDelegate(typeof(Action<IntPtr, byte, int>));
        }
        public unsafe static void memset(void* array, byte what, int length)
        {
            memsetDelegate((IntPtr)array, what, length);
        }
    }


}
