﻿//------------------------------------------------------------------------------
// <auto-generated>
//     This code was generated by a tool.
//     Runtime Version:4.0.30319.42000
//
//     Changes to this file may cause incorrect behavior and will be lost if
//     the code is regenerated.
// </auto-generated>
//------------------------------------------------------------------------------

namespace UniversalRobotsConnect {
    using System;
    
    
    /// <summary>
    ///   A strongly-typed resource class, for looking up localized strings, etc.
    /// </summary>
    // This class was auto-generated by the StronglyTypedResourceBuilder
    // class via a tool like ResGen or Visual Studio.
    // To add or remove a member, edit your .ResX file then rerun ResGen
    // with the /str option, or rebuild your VS project.
    [global::System.CodeDom.Compiler.GeneratedCodeAttribute("System.Resources.Tools.StronglyTypedResourceBuilder", "4.0.0.0")]
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
    [global::System.Runtime.CompilerServices.CompilerGeneratedAttribute()]
    internal class Resources {
        
        private static global::System.Resources.ResourceManager resourceMan;
        
        private static global::System.Globalization.CultureInfo resourceCulture;
        
        [global::System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        internal Resources() {
        }
        
        /// <summary>
        ///   Returns the cached ResourceManager instance used by this class.
        /// </summary>
        [global::System.ComponentModel.EditorBrowsableAttribute(global::System.ComponentModel.EditorBrowsableState.Advanced)]
        internal static global::System.Resources.ResourceManager ResourceManager {
            get {
                if (object.ReferenceEquals(resourceMan, null)) {
                    global::System.Resources.ResourceManager temp = new global::System.Resources.ResourceManager("UniversalRobotsConnect.Resources", typeof(Resources).Assembly);
                    resourceMan = temp;
                }
                return resourceMan;
            }
        }
        
        /// <summary>
        ///   Overrides the current thread's CurrentUICulture property for all
        ///   resource lookups using this strongly typed resource class.
        /// </summary>
        [global::System.ComponentModel.EditorBrowsableAttribute(global::System.ComponentModel.EditorBrowsableState.Advanced)]
        internal static global::System.Globalization.CultureInfo Culture {
            get {
                return resourceCulture;
            }
            set {
                resourceCulture = value;
            }
        }
        
        /// <summary>
        ///   Looks up a localized string similar to &lt;log4net&gt;
        ///  &lt;!-- A1 is set to be a ConsoleAppender --&gt;
        ///  &lt;appender name=&quot;ConsoleAppender&quot; type=&quot;log4net.Appender.ConsoleAppender&quot;&gt;
        ///
        ///    &lt;!-- A1 uses PatternLayout --&gt;
        ///    &lt;layout type=&quot;log4net.Layout.PatternLayout&quot;&gt;
        ///      &lt;conversionPattern value=&quot;%-4timestamp [%thread] %-5level %logger %ndc - %message%newline&quot; /&gt;
        ///    &lt;/layout&gt;
        ///  &lt;/appender&gt;
        ///
        ///  &lt;appender name=&quot;eventFileLog&quot; type=&quot;log4net.Appender.RollingFileAppender&quot;&gt;
        ///    &lt;filter type=&quot;log4net.Filter.LoggerMatchFilter&quot;&gt;
        ///      &lt;loggerToMatch valu [rest of string was truncated]&quot;;.
        /// </summary>
        internal static string logConfig {
            get {
                return ResourceManager.GetString("logConfig", resourceCulture);
            }
        }
        
        /// <summary>
        ///   Looks up a localized string similar to &lt;?xml version=&quot;1.0&quot;?&gt;
        ///&lt;rtde_config&gt;
        ///  &lt;receive key=&quot;out&quot;&gt;
        ///    &lt;!--&lt;field name=&quot;output_bit_registers0_to_31&quot; type=&quot;UINT32&quot;/&gt;
        ///    &lt;field name=&quot;output_int_register_0&quot; type=&quot;INT32&quot;/&gt;
        ///    &lt;field name=&quot;output_int_register_1&quot; type=&quot;INT32&quot;/&gt;
        ///    &lt;field name=&quot;output_double_register_0&quot; type=&quot;DOUBLE&quot;/&gt;
        ///    &lt;field name=&quot;output_double_register_1&quot; type=&quot;DOUBLE&quot;/&gt;--&gt;
        ///
        ///    &lt;field name=&quot;target_q&quot; type=&quot;VECTOR6D&quot;/&gt;
        ///    &lt;field name=&quot;target_qd&quot; type=&quot;VECTOR6D&quot;/&gt;
        ///    &lt;field name=&quot;target_qdd&quot; type=&quot;VECTOR6D&quot;/&gt;
        ///    &lt;f [rest of string was truncated]&quot;;.
        /// </summary>
        internal static string rtde_configuration {
            get {
                return ResourceManager.GetString("rtde_configuration", resourceCulture);
            }
        }
    }
}
