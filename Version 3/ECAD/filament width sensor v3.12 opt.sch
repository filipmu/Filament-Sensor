<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="7.6.0">
<drawing>
<settings>
<setting alwaysvectorfont="yes"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.1" unitdist="inch" unit="inch" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="1" name="Top" color="4" fill="1" visible="no" active="no"/>
<layer number="16" name="Bottom" color="1" fill="1" visible="no" active="no"/>
<layer number="17" name="Pads" color="2" fill="1" visible="no" active="no"/>
<layer number="18" name="Vias" color="2" fill="1" visible="no" active="no"/>
<layer number="19" name="Unrouted" color="6" fill="1" visible="no" active="no"/>
<layer number="20" name="Dimension" color="15" fill="1" visible="no" active="no"/>
<layer number="21" name="tPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="22" name="bPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="23" name="tOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="24" name="bOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="25" name="tNames" color="7" fill="1" visible="no" active="no"/>
<layer number="26" name="bNames" color="7" fill="1" visible="no" active="no"/>
<layer number="27" name="tValues" color="7" fill="1" visible="no" active="no"/>
<layer number="28" name="bValues" color="7" fill="1" visible="no" active="no"/>
<layer number="29" name="tStop" color="7" fill="3" visible="no" active="no"/>
<layer number="30" name="bStop" color="7" fill="6" visible="no" active="no"/>
<layer number="31" name="tCream" color="7" fill="4" visible="no" active="no"/>
<layer number="32" name="bCream" color="7" fill="5" visible="no" active="no"/>
<layer number="33" name="tFinish" color="6" fill="3" visible="no" active="no"/>
<layer number="34" name="bFinish" color="6" fill="6" visible="no" active="no"/>
<layer number="35" name="tGlue" color="7" fill="4" visible="no" active="no"/>
<layer number="36" name="bGlue" color="7" fill="5" visible="no" active="no"/>
<layer number="37" name="tTest" color="7" fill="1" visible="no" active="no"/>
<layer number="38" name="bTest" color="7" fill="1" visible="no" active="no"/>
<layer number="39" name="tKeepout" color="4" fill="11" visible="no" active="no"/>
<layer number="40" name="bKeepout" color="1" fill="11" visible="no" active="no"/>
<layer number="41" name="tRestrict" color="4" fill="10" visible="no" active="no"/>
<layer number="42" name="bRestrict" color="1" fill="10" visible="no" active="no"/>
<layer number="43" name="vRestrict" color="2" fill="10" visible="no" active="no"/>
<layer number="44" name="Drills" color="7" fill="1" visible="no" active="no"/>
<layer number="45" name="Holes" color="7" fill="1" visible="no" active="no"/>
<layer number="46" name="Milling" color="3" fill="1" visible="no" active="no"/>
<layer number="47" name="Measures" color="7" fill="1" visible="no" active="no"/>
<layer number="48" name="Document" color="7" fill="1" visible="no" active="no"/>
<layer number="49" name="Reference" color="7" fill="1" visible="no" active="no"/>
<layer number="50" name="dxf" color="7" fill="1" visible="no" active="no"/>
<layer number="51" name="tDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="52" name="bDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="53" name="tGND_GNDA" color="7" fill="9" visible="no" active="no"/>
<layer number="54" name="bGND_GNDA" color="1" fill="9" visible="no" active="no"/>
<layer number="56" name="wert" color="7" fill="1" visible="no" active="no"/>
<layer number="57" name="tCAD" color="7" fill="1" visible="no" active="no"/>
<layer number="59" name="tCarbon" color="7" fill="1" visible="no" active="no"/>
<layer number="60" name="bCarbon" color="7" fill="1" visible="no" active="no"/>
<layer number="90" name="Modules" color="7" fill="1" visible="yes" active="yes"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="yes" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="yes" active="yes"/>
<layer number="99" name="SpiceOrder" color="7" fill="1" visible="yes" active="yes"/>
<layer number="100" name="Top CU 2" color="12" fill="1" visible="no" active="yes"/>
<layer number="101" name="Gehäuse" color="7" fill="1" visible="no" active="no"/>
<layer number="102" name="Mittellin" color="7" fill="1" visible="no" active="no"/>
<layer number="103" name="tMap" color="7" fill="1" visible="yes" active="yes"/>
<layer number="104" name="Name" color="7" fill="1" visible="no" active="yes"/>
<layer number="105" name="Beschreib" color="9" fill="1" visible="no" active="no"/>
<layer number="106" name="BGA-Top" color="4" fill="1" visible="no" active="no"/>
<layer number="107" name="BD-Top" color="5" fill="1" visible="no" active="no"/>
<layer number="108" name="tplace-old" color="10" fill="1" visible="yes" active="yes"/>
<layer number="109" name="ref-old" color="11" fill="1" visible="yes" active="yes"/>
<layer number="110" name="fp0" color="7" fill="1" visible="yes" active="yes"/>
<layer number="111" name="LPC17xx" color="7" fill="1" visible="yes" active="yes"/>
<layer number="112" name="tSilk" color="7" fill="1" visible="yes" active="yes"/>
<layer number="113" name="IDFDebug" color="4" fill="1" visible="yes" active="yes"/>
<layer number="114" name="Badge_Outline" color="7" fill="1" visible="yes" active="yes"/>
<layer number="115" name="ReferenceISLANDS" color="7" fill="1" visible="yes" active="yes"/>
<layer number="116" name="Patch_BOT" color="9" fill="4" visible="yes" active="yes"/>
<layer number="118" name="Rect_Pads" color="7" fill="1" visible="yes" active="yes"/>
<layer number="121" name="_tsilk" color="7" fill="1" visible="no" active="yes"/>
<layer number="122" name="_bsilk" color="7" fill="1" visible="no" active="yes"/>
<layer number="123" name="tTestmark" color="7" fill="1" visible="yes" active="yes"/>
<layer number="124" name="bTestmark" color="7" fill="1" visible="yes" active="yes"/>
<layer number="125" name="_tNames" color="7" fill="1" visible="yes" active="yes"/>
<layer number="126" name="_bNames" color="7" fill="1" visible="yes" active="yes"/>
<layer number="127" name="_tValues" color="7" fill="1" visible="yes" active="yes"/>
<layer number="128" name="_bValues" color="7" fill="1" visible="yes" active="yes"/>
<layer number="129" name="Mask" color="7" fill="1" visible="yes" active="yes"/>
<layer number="131" name="tAdjust" color="7" fill="1" visible="yes" active="yes"/>
<layer number="132" name="bAdjust" color="7" fill="1" visible="yes" active="yes"/>
<layer number="144" name="Drill_legend" color="7" fill="1" visible="yes" active="yes"/>
<layer number="150" name="Notes" color="7" fill="1" visible="yes" active="yes"/>
<layer number="151" name="HeatSink" color="14" fill="1" visible="no" active="yes"/>
<layer number="152" name="_bDocu" color="7" fill="1" visible="yes" active="yes"/>
<layer number="153" name="FabDoc1" color="7" fill="1" visible="yes" active="yes"/>
<layer number="154" name="FabDoc2" color="7" fill="1" visible="yes" active="yes"/>
<layer number="155" name="FabDoc3" color="7" fill="1" visible="yes" active="yes"/>
<layer number="199" name="Contour" color="7" fill="1" visible="yes" active="yes"/>
<layer number="200" name="200bmp" color="1" fill="10" visible="no" active="no"/>
<layer number="201" name="201bmp" color="2" fill="10" visible="no" active="yes"/>
<layer number="202" name="202bmp" color="3" fill="10" visible="no" active="yes"/>
<layer number="203" name="203bmp" color="4" fill="10" visible="no" active="yes"/>
<layer number="204" name="204bmp" color="5" fill="10" visible="no" active="yes"/>
<layer number="205" name="205bmp" color="6" fill="10" visible="no" active="yes"/>
<layer number="206" name="206bmp" color="7" fill="10" visible="no" active="yes"/>
<layer number="207" name="207bmp" color="8" fill="10" visible="no" active="yes"/>
<layer number="208" name="208bmp" color="9" fill="10" visible="no" active="yes"/>
<layer number="209" name="209bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="210" name="210bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="211" name="211bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="212" name="212bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="213" name="213bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="214" name="214bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="215" name="215bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="216" name="216bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="217" name="217bmp" color="18" fill="1" visible="no" active="no"/>
<layer number="218" name="218bmp" color="19" fill="1" visible="no" active="no"/>
<layer number="219" name="219bmp" color="20" fill="1" visible="no" active="no"/>
<layer number="220" name="220bmp" color="21" fill="1" visible="no" active="no"/>
<layer number="221" name="221bmp" color="22" fill="1" visible="no" active="no"/>
<layer number="222" name="222bmp" color="23" fill="1" visible="no" active="no"/>
<layer number="223" name="223bmp" color="24" fill="1" visible="no" active="no"/>
<layer number="224" name="224bmp" color="25" fill="1" visible="no" active="no"/>
<layer number="225" name="225bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="226" name="226bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="227" name="227bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="228" name="228bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="229" name="229bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="230" name="230bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="231" name="231bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="248" name="Housing" color="7" fill="1" visible="yes" active="yes"/>
<layer number="249" name="Edge" color="7" fill="1" visible="yes" active="yes"/>
<layer number="250" name="Descript" color="3" fill="1" visible="no" active="no"/>
<layer number="251" name="SMDround" color="12" fill="11" visible="no" active="no"/>
<layer number="254" name="cooling" color="7" fill="1" visible="yes" active="yes"/>
<layer number="255" name="tmp" color="7" fill="1" visible="yes" active="yes"/>
</layers>
<schematic>
<libraries>
<library name="filip">
<packages>
<package name="MA03-2">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<wire x1="-3.175" y1="2.54" x2="-1.905" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="2.54" x2="-1.27" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="1.905" x2="-0.635" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="2.54" x2="0.635" y2="2.54" width="0.1524" layer="21"/>
<wire x1="0.635" y1="2.54" x2="1.27" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="2.54" x2="-3.81" y2="1.905" width="0.1524" layer="21"/>
<wire x1="1.27" y1="1.905" x2="1.905" y2="2.54" width="0.1524" layer="21"/>
<wire x1="1.905" y1="2.54" x2="3.175" y2="2.54" width="0.1524" layer="21"/>
<wire x1="3.175" y1="2.54" x2="3.81" y2="1.905" width="0.1524" layer="21"/>
<wire x1="3.81" y1="1.905" x2="3.81" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="-1.905" x2="-1.905" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="1.27" y1="-1.905" x2="0.635" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="0.635" y1="-2.54" x2="-0.635" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="-2.54" x2="-1.27" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-3.81" y1="1.905" x2="-3.81" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-3.81" y1="-1.905" x2="-3.175" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="-2.54" x2="-3.175" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="3.81" y1="-1.905" x2="3.175" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-2.54" x2="1.905" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="1.905" y1="-2.54" x2="1.27" y2="-1.905" width="0.1524" layer="21"/>
<pad name="1" x="-2.54" y="-1.27" drill="0.8" diameter="1.524"/>
<pad name="3" x="0" y="-1.27" drill="0.8" diameter="1.524" shape="octagon"/>
<pad name="5" x="2.54" y="-1.27" drill="0.8" diameter="1.524" shape="octagon"/>
<pad name="2" x="-2.54" y="1.27" drill="0.8" diameter="1.524" shape="octagon"/>
<pad name="4" x="0" y="1.27" drill="0.8" diameter="1.524" shape="octagon"/>
<pad name="6" x="2.54" y="1.27" drill="0.8" diameter="1.524" shape="octagon"/>
<text x="-3.175" y="-3.556" size="0.8128" layer="21" ratio="10">1</text>
<text x="-3.81" y="2.921" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.27" y="-4.191" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-0.254" y1="-1.524" x2="0.254" y2="-1.016" layer="51"/>
<rectangle x1="-2.794" y1="-1.524" x2="-2.286" y2="-1.016" layer="51"/>
<rectangle x1="2.286" y1="-1.524" x2="2.794" y2="-1.016" layer="51"/>
<rectangle x1="-2.794" y1="1.016" x2="-2.286" y2="1.524" layer="51"/>
<rectangle x1="-0.254" y1="1.016" x2="0.254" y2="1.524" layer="51"/>
<rectangle x1="2.286" y1="1.016" x2="2.794" y2="1.524" layer="51"/>
</package>
<package name="2,8">
<description>&lt;b&gt;MOUNTING HOLE&lt;/b&gt; 2.8 mm with drill center</description>
<wire x1="-1.778" y1="0" x2="0" y2="-1.778" width="2.286" layer="51" curve="90" cap="flat"/>
<wire x1="0" y1="1.778" x2="1.778" y2="0" width="2.286" layer="51" curve="-90" cap="flat"/>
<circle x="0" y="0" radius="0.635" width="0.4572" layer="51"/>
<circle x="0" y="0" radius="2.921" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="2.54" width="2.032" layer="40"/>
<circle x="0" y="0" radius="2.54" width="2.032" layer="39"/>
<circle x="0" y="0" radius="2.54" width="2.032" layer="41"/>
<circle x="0" y="0" radius="2.54" width="2.032" layer="42"/>
<circle x="0" y="0" radius="2.54" width="2.032" layer="43"/>
<circle x="0" y="0" radius="1.5" width="0.2032" layer="21"/>
<hole x="0" y="0" drill="2.8"/>
</package>
<package name="3,0">
<description>&lt;b&gt;MOUNTING HOLE&lt;/b&gt; 3.0 mm with drill center</description>
<wire x1="-2.159" y1="0" x2="0" y2="-2.159" width="2.4892" layer="51" curve="90" cap="flat"/>
<wire x1="0" y1="2.159" x2="2.159" y2="0" width="2.4892" layer="51" curve="-90" cap="flat"/>
<circle x="0" y="0" radius="3.429" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="0.762" width="0.4572" layer="51"/>
<circle x="0" y="0" radius="3.048" width="2.032" layer="39"/>
<circle x="0" y="0" radius="3.048" width="2.032" layer="43"/>
<circle x="0" y="0" radius="3.048" width="2.032" layer="40"/>
<circle x="0" y="0" radius="3.048" width="2.032" layer="41"/>
<circle x="0" y="0" radius="3.048" width="2.032" layer="42"/>
<circle x="0" y="0" radius="1.6" width="0.2032" layer="21"/>
<text x="-1.27" y="-3.81" size="1.27" layer="48">3,0</text>
<hole x="0" y="0" drill="3"/>
</package>
<package name="3,3">
<description>&lt;b&gt;MOUNTING HOLE&lt;/b&gt; 3.3 mm with drill center</description>
<wire x1="-2.159" y1="0" x2="0" y2="-2.159" width="2.4892" layer="51" curve="90" cap="flat"/>
<wire x1="0" y1="2.159" x2="2.159" y2="0" width="2.4892" layer="51" curve="-90" cap="flat"/>
<circle x="0" y="0" radius="3.429" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="0.762" width="0.4572" layer="51"/>
<circle x="0" y="0" radius="3.048" width="2.54" layer="39"/>
<circle x="0" y="0" radius="3.048" width="2.54" layer="40"/>
<circle x="0" y="0" radius="3.048" width="2.032" layer="43"/>
<circle x="0" y="0" radius="3.048" width="2.032" layer="41"/>
<circle x="0" y="0" radius="3.048" width="2.032" layer="42"/>
<circle x="0" y="0" radius="1.75" width="0.2032" layer="21"/>
<hole x="0" y="0" drill="3.3"/>
</package>
<package name="3,6">
<description>&lt;b&gt;MOUNTING HOLE&lt;/b&gt; 3.6 mm with drill center</description>
<wire x1="-2.159" y1="0" x2="0" y2="-2.159" width="2.4892" layer="51" curve="90" cap="flat"/>
<wire x1="0" y1="2.159" x2="2.159" y2="0" width="2.4892" layer="51" curve="-90" cap="flat"/>
<circle x="0" y="0" radius="3.429" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="0.762" width="0.4572" layer="51"/>
<circle x="0" y="0" radius="3.048" width="2.7686" layer="39"/>
<circle x="0" y="0" radius="3.048" width="2.7686" layer="40"/>
<circle x="0" y="0" radius="3.048" width="2.032" layer="43"/>
<circle x="0" y="0" radius="3.048" width="2.032" layer="41"/>
<circle x="0" y="0" radius="3.048" width="2.032" layer="42"/>
<circle x="0" y="0" radius="1.9" width="0.2032" layer="21"/>
<hole x="0" y="0" drill="3.6"/>
</package>
<package name="4,1">
<description>&lt;b&gt;MOUNTING HOLE&lt;/b&gt; 4.1 mm with drill center</description>
<wire x1="-2.54" y1="0" x2="0" y2="-2.54" width="3.9116" layer="51" curve="90" cap="flat"/>
<wire x1="0" y1="2.54" x2="2.54" y2="0" width="3.9116" layer="51" curve="-90" cap="flat"/>
<circle x="0" y="0" radius="4.4958" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="0.762" width="0.4572" layer="51"/>
<circle x="0" y="0" radius="3.81" width="2.54" layer="39"/>
<circle x="0" y="0" radius="3.81" width="2.54" layer="40"/>
<circle x="0" y="0" radius="3.81" width="2.54" layer="41"/>
<circle x="0" y="0" radius="3.81" width="2.54" layer="42"/>
<circle x="0" y="0" radius="3.81" width="2.54" layer="43"/>
<circle x="0" y="0" radius="2.15" width="0.1524" layer="21"/>
<hole x="0" y="0" drill="4.1"/>
</package>
<package name="4,5">
<description>&lt;b&gt;MOUNTING HOLE&lt;/b&gt; 4.5 mm with drill center</description>
<wire x1="4.445" y1="0" x2="2.159" y2="0" width="0.0508" layer="21"/>
<wire x1="0" y1="4.445" x2="0" y2="2.159" width="0.0508" layer="21"/>
<wire x1="-2.159" y1="0" x2="-4.445" y2="0" width="0.0508" layer="21"/>
<wire x1="0" y1="-2.159" x2="0" y2="-4.445" width="0.0508" layer="21"/>
<wire x1="-2.54" y1="0" x2="0" y2="-2.54" width="3.9116" layer="51" curve="90" cap="flat"/>
<wire x1="0" y1="2.54" x2="2.54" y2="0" width="3.9116" layer="51" curve="-90" cap="flat"/>
<circle x="0" y="0" radius="4.4958" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="0.762" width="0.4572" layer="51"/>
<circle x="0" y="0" radius="4.699" width="4.5466" layer="39"/>
<circle x="0" y="0" radius="4.699" width="4.5466" layer="40"/>
<circle x="0" y="0" radius="3.8184" width="2.54" layer="41"/>
<circle x="0" y="0" radius="3.81" width="2.54" layer="42"/>
<circle x="0" y="0" radius="3.81" width="2.54" layer="43"/>
<circle x="0" y="0" radius="2.35" width="0.1524" layer="21"/>
<text x="-1.27" y="-4.445" size="1.27" layer="48">4,5</text>
<hole x="0" y="0" drill="4.5"/>
</package>
<package name="5,0">
<description>&lt;b&gt;MOUNTING HOLE&lt;/b&gt; 5.0 mm with drill center</description>
<wire x1="4.445" y1="0" x2="2.159" y2="0" width="0.0508" layer="21"/>
<wire x1="0" y1="4.445" x2="0" y2="2.159" width="0.0508" layer="21"/>
<wire x1="-2.159" y1="0" x2="-4.445" y2="0" width="0.0508" layer="21"/>
<wire x1="0" y1="-2.159" x2="0" y2="-4.445" width="0.0508" layer="21"/>
<wire x1="-2.54" y1="0" x2="0" y2="-2.54" width="3.9116" layer="51" curve="90" cap="flat"/>
<wire x1="0" y1="2.54" x2="2.54" y2="0" width="3.9116" layer="51" curve="-90" cap="flat"/>
<circle x="0" y="0" radius="4.4958" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="0.762" width="0.4572" layer="51"/>
<circle x="0" y="0" radius="4.699" width="4.5466" layer="39"/>
<circle x="0" y="0" radius="4.699" width="4.5466" layer="40"/>
<circle x="0" y="0" radius="4.826" width="2.54" layer="43"/>
<circle x="0" y="0" radius="4.826" width="2.54" layer="42"/>
<circle x="0" y="0" radius="4.826" width="2.54" layer="41"/>
<circle x="0" y="0" radius="2.6" width="0.1524" layer="21"/>
<text x="-1.27" y="-4.445" size="1.27" layer="48">5,0</text>
<hole x="0" y="0" drill="5"/>
</package>
<package name="3,2">
<description>&lt;b&gt;MOUNTING HOLE&lt;/b&gt; 3.2 mm with drill center</description>
<wire x1="-2.159" y1="0" x2="0" y2="-2.159" width="2.4892" layer="51" curve="90" cap="flat"/>
<wire x1="0" y1="2.159" x2="2.159" y2="0" width="2.4892" layer="51" curve="-90" cap="flat"/>
<circle x="0" y="0" radius="3.429" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="0.762" width="0.4572" layer="51"/>
<circle x="0" y="0" radius="3.048" width="2.032" layer="39"/>
<circle x="0" y="0" radius="3.048" width="2.032" layer="43"/>
<circle x="0" y="0" radius="3.048" width="2.032" layer="40"/>
<circle x="0" y="0" radius="3.048" width="2.032" layer="41"/>
<circle x="0" y="0" radius="3.048" width="2.032" layer="42"/>
<circle x="0" y="0" radius="1.7" width="0.2032" layer="21"/>
<hole x="0" y="0" drill="3.2"/>
</package>
<package name="4,3">
<description>&lt;b&gt;MOUNTING HOLE&lt;/b&gt; 4.3 mm with drill center</description>
<wire x1="-2.54" y1="0" x2="0" y2="-2.54" width="3.9116" layer="51" curve="90" cap="flat"/>
<wire x1="0" y1="2.54" x2="2.54" y2="0" width="3.9116" layer="51" curve="-90" cap="flat"/>
<circle x="0" y="0" radius="4.4958" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="0.762" width="0.1524" layer="51"/>
<circle x="0" y="0" radius="4.699" width="4.5466" layer="39"/>
<circle x="0" y="0" radius="4.699" width="4.5466" layer="40"/>
<circle x="0" y="0" radius="3.8184" width="2.54" layer="41"/>
<circle x="0" y="0" radius="3.81" width="2.54" layer="42"/>
<circle x="0" y="0" radius="3.81" width="2.54" layer="43"/>
<circle x="0" y="0" radius="2.25" width="0.1524" layer="21"/>
<hole x="0" y="0" drill="4.3"/>
</package>
<package name="5,5">
<description>&lt;b&gt;MOUNTING HOLE&lt;/b&gt; 5.5 mm with drill center</description>
<wire x1="4.445" y1="0" x2="2.159" y2="0" width="0.0508" layer="21"/>
<wire x1="0" y1="4.445" x2="0" y2="2.159" width="0.0508" layer="21"/>
<wire x1="-2.159" y1="0" x2="-4.445" y2="0" width="0.0508" layer="21"/>
<wire x1="0" y1="-2.159" x2="0" y2="-4.445" width="0.0508" layer="21"/>
<wire x1="-2.54" y1="0" x2="0" y2="-2.54" width="3.9116" layer="51" curve="90" cap="flat"/>
<wire x1="0" y1="2.54" x2="2.54" y2="0" width="3.9116" layer="51" curve="-90" cap="flat"/>
<circle x="0" y="0" radius="4.4958" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="0.762" width="0.4572" layer="51"/>
<circle x="0" y="0" radius="4.699" width="4.5466" layer="39"/>
<circle x="0" y="0" radius="4.699" width="4.5466" layer="40"/>
<circle x="0" y="0" radius="4.826" width="2.54" layer="43"/>
<circle x="0" y="0" radius="4.826" width="2.54" layer="42"/>
<circle x="0" y="0" radius="4.826" width="2.54" layer="41"/>
<circle x="0" y="0" radius="2.85" width="0.1524" layer="21"/>
<hole x="0" y="0" drill="5.5"/>
</package>
<package name="1.5">
<wire x1="-0.778" y1="0" x2="0" y2="-0.778" width="2.286" layer="51" curve="90" cap="flat"/>
<wire x1="0" y1="0.778" x2="0.778" y2="0" width="2.286" layer="51" curve="-90" cap="flat"/>
<circle x="0" y="0" radius="0.635" width="0.4572" layer="51"/>
<circle x="0" y="0" radius="1.5" width="2.032" layer="40"/>
<circle x="0" y="0" radius="1.5811" width="2.032" layer="39"/>
<circle x="0" y="0" radius="1.5811" width="2.032" layer="41"/>
<circle x="0" y="0" radius="1.5811" width="2.032" layer="42"/>
<circle x="0" y="0" radius="1.5811" width="2.032" layer="43"/>
<circle x="0" y="0" radius="1.5" width="0.2032" layer="21"/>
<hole x="0" y="0" drill="1.5"/>
</package>
<package name="2.0">
<wire x1="-0.778" y1="0" x2="0" y2="-0.778" width="2.286" layer="51" curve="90" cap="flat"/>
<wire x1="0" y1="0.778" x2="0.778" y2="0" width="2.286" layer="51" curve="-90" cap="flat"/>
<circle x="0" y="0" radius="0.635" width="0.4572" layer="51"/>
<circle x="0" y="0" radius="2" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="1.118" width="2.032" layer="40"/>
<circle x="0" y="0" radius="1" width="2.032" layer="39"/>
<circle x="0" y="0" radius="1" width="2.032" layer="41"/>
<circle x="0" y="0" radius="1.118" width="2.032" layer="42"/>
<circle x="0" y="0" radius="1" width="2.032" layer="43"/>
<circle x="0" y="0" radius="1.118" width="0.2032" layer="21"/>
<hole x="0" y="0" drill="2"/>
</package>
<package name="SCREWHOLE2-56">
<description>2-56 screw hole</description>
<circle x="0" y="0" radius="1.2446" width="0.2032" layer="21"/>
<hole x="0" y="0" drill="2.1844"/>
</package>
<package name="1X02">
<wire x1="-0.635" y1="1.27" x2="0.635" y2="1.27" width="0.2032" layer="21"/>
<wire x1="0.635" y1="1.27" x2="1.27" y2="0.635" width="0.2032" layer="21"/>
<wire x1="1.27" y1="-0.635" x2="0.635" y2="-1.27" width="0.2032" layer="21"/>
<wire x1="1.27" y1="0.635" x2="1.905" y2="1.27" width="0.2032" layer="21"/>
<wire x1="1.905" y1="1.27" x2="3.175" y2="1.27" width="0.2032" layer="21"/>
<wire x1="3.175" y1="1.27" x2="3.81" y2="0.635" width="0.2032" layer="21"/>
<wire x1="3.81" y1="-0.635" x2="3.175" y2="-1.27" width="0.2032" layer="21"/>
<wire x1="3.175" y1="-1.27" x2="1.905" y2="-1.27" width="0.2032" layer="21"/>
<wire x1="1.905" y1="-1.27" x2="1.27" y2="-0.635" width="0.2032" layer="21"/>
<wire x1="-1.27" y1="0.635" x2="-1.27" y2="-0.635" width="0.2032" layer="21"/>
<wire x1="-0.635" y1="1.27" x2="-1.27" y2="0.635" width="0.2032" layer="21"/>
<wire x1="-1.27" y1="-0.635" x2="-0.635" y2="-1.27" width="0.2032" layer="21"/>
<wire x1="0.635" y1="-1.27" x2="-0.635" y2="-1.27" width="0.2032" layer="21"/>
<wire x1="3.81" y1="0.635" x2="3.81" y2="-0.635" width="0.2032" layer="21"/>
<pad name="1" x="0" y="0" drill="1.016" diameter="1.8796" rot="R90"/>
<pad name="2" x="2.54" y="0" drill="1.016" diameter="1.8796" rot="R90"/>
<text x="-1.3462" y="1.8288" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.27" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="2.286" y1="-0.254" x2="2.794" y2="0.254" layer="51"/>
<rectangle x1="-0.254" y1="-0.254" x2="0.254" y2="0.254" layer="51"/>
</package>
<package name="MOLEX-1X2">
<wire x1="-1.27" y1="3.048" x2="-1.27" y2="-2.54" width="0.127" layer="21"/>
<wire x1="3.81" y1="3.048" x2="3.81" y2="-2.54" width="0.127" layer="21"/>
<wire x1="3.81" y1="3.048" x2="-1.27" y2="3.048" width="0.127" layer="21"/>
<wire x1="3.81" y1="-2.54" x2="2.54" y2="-2.54" width="0.127" layer="21"/>
<wire x1="2.54" y1="-2.54" x2="0" y2="-2.54" width="0.127" layer="21"/>
<wire x1="0" y1="-2.54" x2="-1.27" y2="-2.54" width="0.127" layer="21"/>
<wire x1="0" y1="-2.54" x2="0" y2="-1.27" width="0.127" layer="21"/>
<wire x1="0" y1="-1.27" x2="2.54" y2="-1.27" width="0.127" layer="21"/>
<wire x1="2.54" y1="-1.27" x2="2.54" y2="-2.54" width="0.127" layer="21"/>
<pad name="1" x="0" y="0" drill="1.016" diameter="1.8796" shape="square"/>
<pad name="2" x="2.54" y="0" drill="1.016" diameter="1.8796"/>
</package>
<package name="SCREWTERMINAL-3.5MM-2">
<wire x1="-1.75" y1="3.4" x2="5.25" y2="3.4" width="0.2032" layer="21"/>
<wire x1="5.25" y1="3.4" x2="5.25" y2="-2.8" width="0.2032" layer="21"/>
<wire x1="5.25" y1="-2.8" x2="5.25" y2="-3.6" width="0.2032" layer="21"/>
<wire x1="5.25" y1="-3.6" x2="-1.75" y2="-3.6" width="0.2032" layer="21"/>
<wire x1="-1.75" y1="-3.6" x2="-1.75" y2="-2.8" width="0.2032" layer="21"/>
<wire x1="-1.75" y1="-2.8" x2="-1.75" y2="3.4" width="0.2032" layer="21"/>
<wire x1="5.25" y1="-2.8" x2="-1.75" y2="-2.8" width="0.2032" layer="21"/>
<wire x1="-1.75" y1="-1.35" x2="-2.15" y2="-1.35" width="0.2032" layer="51"/>
<wire x1="-2.15" y1="-1.35" x2="-2.15" y2="-2.35" width="0.2032" layer="51"/>
<wire x1="-2.15" y1="-2.35" x2="-1.75" y2="-2.35" width="0.2032" layer="51"/>
<wire x1="5.25" y1="3.15" x2="5.65" y2="3.15" width="0.2032" layer="51"/>
<wire x1="5.65" y1="3.15" x2="5.65" y2="2.15" width="0.2032" layer="51"/>
<wire x1="5.65" y1="2.15" x2="5.25" y2="2.15" width="0.2032" layer="51"/>
<circle x="2" y="3" radius="0.2828" width="0.127" layer="51"/>
<pad name="1" x="0" y="0" drill="1.2" diameter="2.032" shape="square"/>
<pad name="2" x="3.5" y="0" drill="1.2" diameter="2.032"/>
<text x="-1.27" y="2.54" size="0.4064" layer="25">&gt;NAME</text>
<text x="-1.27" y="1.27" size="0.4064" layer="27">&gt;VALUE</text>
</package>
<package name="JST-2-SMD">
<description>2mm SMD side-entry connector. tDocu layer indicates the actual physical plastic housing. +/- indicate SparkFun standard batteries and wiring.</description>
<wire x1="-4" y1="-1" x2="-4" y2="-4.5" width="0.2032" layer="21"/>
<wire x1="-4" y1="-4.5" x2="-3.2" y2="-4.5" width="0.2032" layer="21"/>
<wire x1="-3.2" y1="-4.5" x2="-3.2" y2="-2" width="0.2032" layer="21"/>
<wire x1="-3.2" y1="-2" x2="-2" y2="-2" width="0.2032" layer="21"/>
<wire x1="2" y1="-2" x2="3.2" y2="-2" width="0.2032" layer="21"/>
<wire x1="3.2" y1="-2" x2="3.2" y2="-4.5" width="0.2032" layer="21"/>
<wire x1="3.2" y1="-4.5" x2="4" y2="-4.5" width="0.2032" layer="21"/>
<wire x1="4" y1="-4.5" x2="4" y2="-1" width="0.2032" layer="21"/>
<wire x1="2" y1="3" x2="-2" y2="3" width="0.2032" layer="21"/>
<smd name="1" x="-1" y="-3.7" dx="1" dy="4.6" layer="1"/>
<smd name="2" x="1" y="-3.7" dx="1" dy="4.6" layer="1"/>
<smd name="NC1" x="-3.4" y="1.5" dx="3.4" dy="1.6" layer="1" rot="R90"/>
<smd name="NC2" x="3.4" y="1.5" dx="3.4" dy="1.6" layer="1" rot="R90"/>
<text x="-1.27" y="1.27" size="0.4064" layer="25">&gt;Name</text>
<text x="-1.27" y="0" size="0.4064" layer="27">&gt;Value</text>
<text x="2.159" y="-4.445" size="1.27" layer="51">+</text>
<text x="-2.921" y="-4.445" size="1.27" layer="51">-</text>
</package>
<package name="1X02_BIG">
<wire x1="-1.27" y1="1.27" x2="-1.27" y2="-1.27" width="0.127" layer="21"/>
<wire x1="-1.27" y1="-1.27" x2="5.08" y2="-1.27" width="0.127" layer="21"/>
<wire x1="5.08" y1="-1.27" x2="5.08" y2="1.27" width="0.127" layer="21"/>
<wire x1="5.08" y1="1.27" x2="-1.27" y2="1.27" width="0.127" layer="21"/>
<pad name="P$1" x="0" y="0" drill="1.0668"/>
<pad name="P$2" x="3.81" y="0" drill="1.0668"/>
</package>
<package name="JST-2-SMD-VERT">
<wire x1="-4.1" y1="2.97" x2="4.2" y2="2.97" width="0.2032" layer="51"/>
<wire x1="4.2" y1="2.97" x2="4.2" y2="-2.13" width="0.2032" layer="51"/>
<wire x1="4.2" y1="-2.13" x2="-4.1" y2="-2.13" width="0.2032" layer="51"/>
<wire x1="-4.1" y1="-2.13" x2="-4.1" y2="2.97" width="0.2032" layer="51"/>
<wire x1="-4.1" y1="3" x2="4.2" y2="3" width="0.2032" layer="21"/>
<wire x1="4.2" y1="3" x2="4.2" y2="2.3" width="0.2032" layer="21"/>
<wire x1="-4.1" y1="3" x2="-4.1" y2="2.3" width="0.2032" layer="21"/>
<wire x1="2" y1="-2.1" x2="4.2" y2="-2.1" width="0.2032" layer="21"/>
<wire x1="4.2" y1="-2.1" x2="4.2" y2="-1.7" width="0.2032" layer="21"/>
<wire x1="-2" y1="-2.1" x2="-4.1" y2="-2.1" width="0.2032" layer="21"/>
<wire x1="-4.1" y1="-2.1" x2="-4.1" y2="-1.8" width="0.2032" layer="21"/>
<smd name="P$1" x="-3.4" y="0.27" dx="3" dy="1.6" layer="1" rot="R90"/>
<smd name="P$2" x="3.4" y="0.27" dx="3" dy="1.6" layer="1" rot="R90"/>
<smd name="VCC" x="-1" y="-2" dx="1" dy="5.5" layer="1"/>
<smd name="GND" x="1" y="-2" dx="1" dy="5.5" layer="1"/>
<text x="2.54" y="-5.08" size="1.27" layer="25">&gt;Name</text>
<text x="2.24" y="3.48" size="1.27" layer="27">&gt;Value</text>
</package>
<package name="R_SW_TH">
<wire x1="-1.651" y1="19.2532" x2="-1.651" y2="-1.3716" width="0.2032" layer="21"/>
<wire x1="-1.651" y1="-1.3716" x2="-1.651" y2="-2.2352" width="0.2032" layer="21"/>
<wire x1="-1.651" y1="19.2532" x2="13.589" y2="19.2532" width="0.2032" layer="21"/>
<wire x1="13.589" y1="19.2532" x2="13.589" y2="-2.2352" width="0.2032" layer="21"/>
<wire x1="13.589" y1="-2.2352" x2="-1.651" y2="-2.2352" width="0.2032" layer="21"/>
<pad name="P$1" x="0" y="0" drill="1.6002"/>
<pad name="P$2" x="0" y="16.9926" drill="1.6002"/>
<pad name="P$3" x="12.0904" y="15.494" drill="1.6002"/>
<pad name="P$4" x="12.0904" y="8.4582" drill="1.6002"/>
</package>
<package name="SCREWTERMINAL-5MM-2">
<wire x1="-3.1" y1="4.2" x2="8.1" y2="4.2" width="0.2032" layer="21"/>
<wire x1="8.1" y1="4.2" x2="8.1" y2="-2.3" width="0.2032" layer="21"/>
<wire x1="8.1" y1="-2.3" x2="8.1" y2="-3.3" width="0.2032" layer="21"/>
<wire x1="8.1" y1="-3.3" x2="-3.1" y2="-3.3" width="0.2032" layer="21"/>
<wire x1="-3.1" y1="-3.3" x2="-3.1" y2="-2.3" width="0.2032" layer="21"/>
<wire x1="-3.1" y1="-2.3" x2="-3.1" y2="4.2" width="0.2032" layer="21"/>
<wire x1="8.1" y1="-2.3" x2="-3.1" y2="-2.3" width="0.2032" layer="21"/>
<wire x1="-3.1" y1="-1.35" x2="-3.7" y2="-1.35" width="0.2032" layer="51"/>
<wire x1="-3.7" y1="-1.35" x2="-3.7" y2="-2.35" width="0.2032" layer="51"/>
<wire x1="-3.7" y1="-2.35" x2="-3.1" y2="-2.35" width="0.2032" layer="51"/>
<wire x1="8.1" y1="4" x2="8.7" y2="4" width="0.2032" layer="51"/>
<wire x1="8.7" y1="4" x2="8.7" y2="3" width="0.2032" layer="51"/>
<wire x1="8.7" y1="3" x2="8.1" y2="3" width="0.2032" layer="51"/>
<circle x="2.5" y="3.7" radius="0.2828" width="0.127" layer="51"/>
<pad name="1" x="0" y="0" drill="1.3" diameter="2.032" shape="square"/>
<pad name="2" x="5" y="0" drill="1.3" diameter="2.032"/>
<text x="-1.27" y="2.54" size="0.4064" layer="25">&gt;NAME</text>
<text x="-1.27" y="1.27" size="0.4064" layer="27">&gt;VALUE</text>
</package>
<package name="1X02_LOCK">
<wire x1="-0.635" y1="1.27" x2="0.635" y2="1.27" width="0.2032" layer="21"/>
<wire x1="0.635" y1="1.27" x2="1.27" y2="0.635" width="0.2032" layer="21"/>
<wire x1="1.27" y1="-0.635" x2="0.635" y2="-1.27" width="0.2032" layer="21"/>
<wire x1="1.27" y1="0.635" x2="1.905" y2="1.27" width="0.2032" layer="21"/>
<wire x1="1.905" y1="1.27" x2="3.175" y2="1.27" width="0.2032" layer="21"/>
<wire x1="3.175" y1="1.27" x2="3.81" y2="0.635" width="0.2032" layer="21"/>
<wire x1="3.81" y1="-0.635" x2="3.175" y2="-1.27" width="0.2032" layer="21"/>
<wire x1="3.175" y1="-1.27" x2="1.905" y2="-1.27" width="0.2032" layer="21"/>
<wire x1="1.905" y1="-1.27" x2="1.27" y2="-0.635" width="0.2032" layer="21"/>
<wire x1="-1.27" y1="0.635" x2="-1.27" y2="-0.635" width="0.2032" layer="21"/>
<wire x1="-0.635" y1="1.27" x2="-1.27" y2="0.635" width="0.2032" layer="21"/>
<wire x1="-1.27" y1="-0.635" x2="-0.635" y2="-1.27" width="0.2032" layer="21"/>
<wire x1="0.635" y1="-1.27" x2="-0.635" y2="-1.27" width="0.2032" layer="21"/>
<wire x1="3.81" y1="0.635" x2="3.81" y2="-0.635" width="0.2032" layer="21"/>
<pad name="1" x="-0.1778" y="0" drill="1.016" diameter="1.8796" rot="R90"/>
<pad name="2" x="2.7178" y="0" drill="1.016" diameter="1.8796" rot="R90"/>
<text x="-1.3462" y="1.8288" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.27" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.2921" y1="-0.2921" x2="0.2921" y2="0.2921" layer="51"/>
<rectangle x1="2.2479" y1="-0.2921" x2="2.8321" y2="0.2921" layer="51"/>
</package>
<package name="MOLEX-1X2_LOCK">
<wire x1="-1.27" y1="3.048" x2="-1.27" y2="-2.54" width="0.127" layer="21"/>
<wire x1="3.81" y1="3.048" x2="3.81" y2="-2.54" width="0.127" layer="21"/>
<wire x1="3.81" y1="3.048" x2="-1.27" y2="3.048" width="0.127" layer="21"/>
<wire x1="3.81" y1="-2.54" x2="2.54" y2="-2.54" width="0.127" layer="21"/>
<wire x1="2.54" y1="-2.54" x2="0" y2="-2.54" width="0.127" layer="21"/>
<wire x1="0" y1="-2.54" x2="-1.27" y2="-2.54" width="0.127" layer="21"/>
<wire x1="0" y1="-2.54" x2="0" y2="-1.27" width="0.127" layer="21"/>
<wire x1="0" y1="-1.27" x2="2.54" y2="-1.27" width="0.127" layer="21"/>
<wire x1="2.54" y1="-1.27" x2="2.54" y2="-2.54" width="0.127" layer="21"/>
<pad name="1" x="-0.127" y="0" drill="1.016" diameter="1.8796" shape="square"/>
<pad name="2" x="2.667" y="0" drill="1.016" diameter="1.8796"/>
<rectangle x1="-0.2921" y1="-0.2921" x2="0.2921" y2="0.2921" layer="51"/>
<rectangle x1="2.2479" y1="-0.2921" x2="2.8321" y2="0.2921" layer="51"/>
</package>
<package name="1X02_LOCK_LONGPADS">
<description>This footprint was designed to help hold the alignment of a through-hole component (i.e.  6-pin header) while soldering it into place.  
You may notice that each hole has been shifted either up or down by 0.005 of an inch from it's more standard position (which is a perfectly straight line).  
This slight alteration caused the pins (the squares in the middle) to touch the edges of the holes.  Because they are alternating, it causes a "brace" 
to hold the component in place.  0.005 has proven to be the perfect amount of "off-center" position when using our standard breakaway headers.
Although looks a little odd when you look at the bare footprint, once you have a header in there, the alteration is very hard to notice.  Also,
if you push a header all the way into place, it is covered up entirely on the bottom side.  This idea of altering the position of holes to aid alignment 
will be further integrated into the Sparkfun Library for other footprints.  It can help hold any component with 3 or more connection pins.</description>
<wire x1="1.651" y1="0" x2="0.889" y2="0" width="0.2032" layer="21"/>
<wire x1="-1.27" y1="0" x2="-1.016" y2="0" width="0.2032" layer="21"/>
<wire x1="-1.27" y1="0" x2="-1.27" y2="0.9906" width="0.2032" layer="21"/>
<wire x1="-1.27" y1="0.9906" x2="-0.9906" y2="1.27" width="0.2032" layer="21"/>
<wire x1="-1.27" y1="0" x2="-1.27" y2="-0.9906" width="0.2032" layer="21"/>
<wire x1="-1.27" y1="-0.9906" x2="-0.9906" y2="-1.27" width="0.2032" layer="21"/>
<wire x1="3.81" y1="0" x2="3.556" y2="0" width="0.2032" layer="21"/>
<wire x1="3.81" y1="0" x2="3.81" y2="-0.9906" width="0.2032" layer="21"/>
<wire x1="3.81" y1="-0.9906" x2="3.5306" y2="-1.27" width="0.2032" layer="21"/>
<wire x1="3.81" y1="0" x2="3.81" y2="0.9906" width="0.2032" layer="21"/>
<wire x1="3.81" y1="0.9906" x2="3.5306" y2="1.27" width="0.2032" layer="21"/>
<pad name="1" x="-0.127" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="2" x="2.667" y="0" drill="1.016" shape="long" rot="R90"/>
<text x="-1.27" y="1.778" size="1.27" layer="25" font="vector">&gt;NAME</text>
<text x="-1.27" y="-3.302" size="1.27" layer="27" font="vector">&gt;VALUE</text>
<rectangle x1="-0.2921" y1="-0.2921" x2="0.2921" y2="0.2921" layer="51"/>
<rectangle x1="2.2479" y1="-0.2921" x2="2.8321" y2="0.2921" layer="51"/>
</package>
<package name="SCREWTERMINAL-3.5MM-2_LOCK">
<wire x1="-1.75" y1="3.4" x2="5.25" y2="3.4" width="0.2032" layer="21"/>
<wire x1="5.25" y1="3.4" x2="5.25" y2="-2.8" width="0.2032" layer="21"/>
<wire x1="5.25" y1="-2.8" x2="5.25" y2="-3.6" width="0.2032" layer="21"/>
<wire x1="5.25" y1="-3.6" x2="-1.75" y2="-3.6" width="0.2032" layer="21"/>
<wire x1="-1.75" y1="-3.6" x2="-1.75" y2="-2.8" width="0.2032" layer="21"/>
<wire x1="-1.75" y1="-2.8" x2="-1.75" y2="3.4" width="0.2032" layer="21"/>
<wire x1="5.25" y1="-2.8" x2="-1.75" y2="-2.8" width="0.2032" layer="21"/>
<wire x1="-1.75" y1="-1.35" x2="-2.15" y2="-1.35" width="0.2032" layer="51"/>
<wire x1="-2.15" y1="-1.35" x2="-2.15" y2="-2.35" width="0.2032" layer="51"/>
<wire x1="-2.15" y1="-2.35" x2="-1.75" y2="-2.35" width="0.2032" layer="51"/>
<wire x1="5.25" y1="3.15" x2="5.65" y2="3.15" width="0.2032" layer="51"/>
<wire x1="5.65" y1="3.15" x2="5.65" y2="2.15" width="0.2032" layer="51"/>
<wire x1="5.65" y1="2.15" x2="5.25" y2="2.15" width="0.2032" layer="51"/>
<circle x="2" y="3" radius="0.2828" width="0.127" layer="51"/>
<circle x="0" y="0" radius="0.4318" width="0.0254" layer="51"/>
<circle x="3.5" y="0" radius="0.4318" width="0.0254" layer="51"/>
<pad name="1" x="-0.1778" y="0" drill="1.2" diameter="2.032" shape="square"/>
<pad name="2" x="3.6778" y="0" drill="1.2" diameter="2.032"/>
<text x="-1.27" y="2.54" size="0.4064" layer="25">&gt;NAME</text>
<text x="-1.27" y="1.27" size="0.4064" layer="27">&gt;VALUE</text>
</package>
<package name="1X02_LONGPADS">
<pad name="1" x="0" y="0" drill="1.1176" diameter="1.8796" shape="long" rot="R90"/>
<pad name="2" x="2.54" y="0" drill="1.1176" diameter="1.8796" shape="long" rot="R90"/>
</package>
<package name="1X02_NO_SILK">
<pad name="1" x="0" y="0" drill="1.016" diameter="1.8796" rot="R90"/>
<pad name="2" x="2.54" y="0" drill="1.016" diameter="1.8796" rot="R90"/>
<text x="-1.3462" y="1.8288" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.27" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="2.286" y1="-0.254" x2="2.794" y2="0.254" layer="51"/>
<rectangle x1="-0.254" y1="-0.254" x2="0.254" y2="0.254" layer="51"/>
</package>
<package name="JST-2-PTH">
<wire x1="-2" y1="0" x2="-2" y2="-1.6" width="0.2032" layer="21"/>
<wire x1="-2" y1="-1.6" x2="-2.95" y2="-1.6" width="0.2032" layer="21"/>
<wire x1="-2.95" y1="-1.6" x2="-2.95" y2="6" width="0.2032" layer="21"/>
<wire x1="-2.95" y1="6" x2="2.95" y2="6" width="0.2032" layer="21"/>
<wire x1="2.95" y1="6" x2="2.95" y2="-1.6" width="0.2032" layer="21"/>
<wire x1="2.95" y1="-1.6" x2="2" y2="-1.6" width="0.2032" layer="21"/>
<wire x1="2" y1="-1.6" x2="2" y2="0" width="0.2032" layer="21"/>
<pad name="1" x="-1" y="0" drill="0.7" diameter="1.4478"/>
<pad name="2" x="1" y="0" drill="0.7" diameter="1.4478"/>
<text x="-1.27" y="5.27" size="0.4064" layer="25">&gt;Name</text>
<text x="-1.27" y="4" size="0.4064" layer="27">&gt;Value</text>
<text x="0.6" y="0.7" size="1.27" layer="51">+</text>
<text x="-1.4" y="0.7" size="1.27" layer="51">-</text>
</package>
<package name="1X02_XTRA_BIG">
<wire x1="-5.08" y1="2.54" x2="-5.08" y2="-2.54" width="0.127" layer="21"/>
<wire x1="-5.08" y1="-2.54" x2="5.08" y2="-2.54" width="0.127" layer="21"/>
<wire x1="5.08" y1="-2.54" x2="5.08" y2="2.54" width="0.127" layer="21"/>
<wire x1="5.08" y1="2.54" x2="-5.08" y2="2.54" width="0.127" layer="21"/>
<pad name="1" x="-2.54" y="0" drill="2.0574" diameter="3.556"/>
<pad name="2" x="2.54" y="0" drill="2.0574" diameter="3.556"/>
</package>
<package name="1X02_PP_HOLES_ONLY">
<circle x="0" y="0" radius="0.635" width="0.127" layer="51"/>
<circle x="2.54" y="0" radius="0.635" width="0.127" layer="51"/>
<pad name="1" x="0" y="0" drill="0.889" diameter="0.8128" rot="R90"/>
<pad name="2" x="2.54" y="0" drill="0.889" diameter="0.8128" rot="R90"/>
<hole x="0" y="0" drill="1.4732"/>
<hole x="2.54" y="0" drill="1.4732"/>
</package>
<package name="SCREWTERMINAL-3.5MM-2-NS">
<wire x1="-1.75" y1="3.4" x2="5.25" y2="3.4" width="0.2032" layer="51"/>
<wire x1="5.25" y1="3.4" x2="5.25" y2="-2.8" width="0.2032" layer="51"/>
<wire x1="5.25" y1="-2.8" x2="5.25" y2="-3.6" width="0.2032" layer="51"/>
<wire x1="5.25" y1="-3.6" x2="-1.75" y2="-3.6" width="0.2032" layer="51"/>
<wire x1="-1.75" y1="-3.6" x2="-1.75" y2="-2.8" width="0.2032" layer="51"/>
<wire x1="-1.75" y1="-2.8" x2="-1.75" y2="3.4" width="0.2032" layer="51"/>
<wire x1="5.25" y1="-2.8" x2="-1.75" y2="-2.8" width="0.2032" layer="51"/>
<wire x1="-1.75" y1="-1.35" x2="-2.15" y2="-1.35" width="0.2032" layer="51"/>
<wire x1="-2.15" y1="-1.35" x2="-2.15" y2="-2.35" width="0.2032" layer="51"/>
<wire x1="-2.15" y1="-2.35" x2="-1.75" y2="-2.35" width="0.2032" layer="51"/>
<wire x1="5.25" y1="3.15" x2="5.65" y2="3.15" width="0.2032" layer="51"/>
<wire x1="5.65" y1="3.15" x2="5.65" y2="2.15" width="0.2032" layer="51"/>
<wire x1="5.65" y1="2.15" x2="5.25" y2="2.15" width="0.2032" layer="51"/>
<circle x="2" y="3" radius="0.2828" width="0.127" layer="51"/>
<pad name="1" x="0" y="0" drill="1.2" diameter="2.032" shape="square"/>
<pad name="2" x="3.5" y="0" drill="1.2" diameter="2.032"/>
<text x="-1.27" y="2.54" size="0.4064" layer="25">&gt;NAME</text>
<text x="-1.27" y="1.27" size="0.4064" layer="27">&gt;VALUE</text>
</package>
<package name="JST-2-PTH-NS">
<wire x1="-2" y1="0" x2="-2" y2="-1.8" width="0.2032" layer="51"/>
<wire x1="-2" y1="-1.8" x2="-3" y2="-1.8" width="0.2032" layer="51"/>
<wire x1="-3" y1="-1.8" x2="-3" y2="6" width="0.2032" layer="51"/>
<wire x1="-3" y1="6" x2="3" y2="6" width="0.2032" layer="51"/>
<wire x1="3" y1="6" x2="3" y2="-1.8" width="0.2032" layer="51"/>
<wire x1="3" y1="-1.8" x2="2" y2="-1.8" width="0.2032" layer="51"/>
<wire x1="2" y1="-1.8" x2="2" y2="0" width="0.2032" layer="51"/>
<pad name="1" x="-1" y="0" drill="0.7" diameter="1.4478"/>
<pad name="2" x="1" y="0" drill="0.7" diameter="1.4478"/>
<text x="-1.27" y="5.27" size="0.4064" layer="25">&gt;Name</text>
<text x="-1.27" y="4" size="0.4064" layer="27">&gt;Value</text>
<text x="0.6" y="0.7" size="1.27" layer="51">+</text>
<text x="-1.4" y="0.7" size="1.27" layer="51">-</text>
</package>
<package name="JST-2-PTH-KIT">
<description>&lt;H3&gt;JST-2-PTH-KIT&lt;/h3&gt;
2-Pin JST, through-hole connector&lt;br&gt;
&lt;br&gt;
&lt;b&gt;Warning:&lt;/b&gt; This is the KIT version of this package. This package has a smaller diameter top stop mask, which doesn't cover the diameter of the pad. This means only the bottom side of the pads' copper will be exposed. You'll only be able to solder to the bottom side.</description>
<wire x1="-2" y1="0" x2="-2" y2="-1.8" width="0.2032" layer="51"/>
<wire x1="-2" y1="-1.8" x2="-3" y2="-1.8" width="0.2032" layer="51"/>
<wire x1="-3" y1="-1.8" x2="-3" y2="6" width="0.2032" layer="51"/>
<wire x1="-3" y1="6" x2="3" y2="6" width="0.2032" layer="51"/>
<wire x1="3" y1="6" x2="3" y2="-1.8" width="0.2032" layer="51"/>
<wire x1="3" y1="-1.8" x2="2" y2="-1.8" width="0.2032" layer="51"/>
<wire x1="2" y1="-1.8" x2="2" y2="0" width="0.2032" layer="51"/>
<pad name="1" x="-1" y="0" drill="0.7" diameter="1.4478" stop="no"/>
<pad name="2" x="1" y="0" drill="0.7" diameter="1.4478" stop="no"/>
<text x="-1.27" y="5.27" size="0.4064" layer="25">&gt;Name</text>
<text x="-1.27" y="4" size="0.4064" layer="27">&gt;Value</text>
<text x="0.6" y="0.7" size="1.27" layer="51">+</text>
<text x="-1.4" y="0.7" size="1.27" layer="51">-</text>
<polygon width="0.127" layer="30">
<vertex x="-0.9975" y="-0.6604" curve="-90.025935"/>
<vertex x="-1.6604" y="0" curve="-90.017354"/>
<vertex x="-1" y="0.6604" curve="-90"/>
<vertex x="-0.3396" y="0" curve="-90.078137"/>
</polygon>
<polygon width="0.127" layer="29">
<vertex x="-1" y="-0.2865" curve="-90.08005"/>
<vertex x="-1.2865" y="0" curve="-90.040011"/>
<vertex x="-1" y="0.2865" curve="-90"/>
<vertex x="-0.7135" y="0" curve="-90"/>
</polygon>
<polygon width="0.127" layer="30">
<vertex x="1.0025" y="-0.6604" curve="-90.025935"/>
<vertex x="0.3396" y="0" curve="-90.017354"/>
<vertex x="1" y="0.6604" curve="-90"/>
<vertex x="1.6604" y="0" curve="-90.078137"/>
</polygon>
<polygon width="0.127" layer="29">
<vertex x="1" y="-0.2865" curve="-90.08005"/>
<vertex x="0.7135" y="0" curve="-90.040011"/>
<vertex x="1" y="0.2865" curve="-90"/>
<vertex x="1.2865" y="0" curve="-90"/>
</polygon>
</package>
<package name="SPRINGTERMINAL-2.54MM-2">
<wire x1="-4.2" y1="7.88" x2="-4.2" y2="-2.8" width="0.254" layer="21"/>
<wire x1="-4.2" y1="-2.8" x2="-4.2" y2="-4.72" width="0.254" layer="51"/>
<wire x1="-4.2" y1="-4.72" x2="3.44" y2="-4.72" width="0.254" layer="51"/>
<wire x1="3.44" y1="-4.72" x2="3.44" y2="-2.8" width="0.254" layer="51"/>
<wire x1="3.44" y1="7.88" x2="-4.2" y2="7.88" width="0.254" layer="21"/>
<wire x1="0" y1="0" x2="0" y2="5.08" width="0.254" layer="1"/>
<wire x1="0" y1="0" x2="0" y2="5.08" width="0.254" layer="16"/>
<wire x1="2.54" y1="0" x2="2.54" y2="5.08" width="0.254" layer="16"/>
<wire x1="2.54" y1="0" x2="2.54" y2="5.08" width="0.254" layer="1"/>
<wire x1="-4.2" y1="-2.8" x2="3.44" y2="-2.8" width="0.254" layer="21"/>
<wire x1="3.44" y1="4" x2="3.44" y2="1" width="0.254" layer="21"/>
<wire x1="3.44" y1="7.88" x2="3.44" y2="6" width="0.254" layer="21"/>
<wire x1="3.44" y1="-0.9" x2="3.44" y2="-2.8" width="0.254" layer="21"/>
<pad name="1" x="0" y="0" drill="1.1" diameter="1.9"/>
<pad name="P$2" x="0" y="5.08" drill="1.1" diameter="1.9"/>
<pad name="P$3" x="2.54" y="5.08" drill="1.1" diameter="1.9"/>
<pad name="2" x="2.54" y="0" drill="1.1" diameter="1.9"/>
</package>
<package name="1X02_2.54_SCREWTERM">
<pad name="P2" x="0" y="0" drill="1.016" shape="square"/>
<pad name="P1" x="2.54" y="0" drill="1.016" shape="square"/>
<wire x1="-1.5" y1="3.25" x2="4" y2="3.25" width="0.127" layer="21"/>
<wire x1="4" y1="3.25" x2="4" y2="2.5" width="0.127" layer="21"/>
<wire x1="4" y1="2.5" x2="4" y2="-3.25" width="0.127" layer="21"/>
<wire x1="4" y1="-3.25" x2="-1.5" y2="-3.25" width="0.127" layer="21"/>
<wire x1="-1.5" y1="-3.25" x2="-1.5" y2="2.5" width="0.127" layer="21"/>
<wire x1="-1.5" y1="2.5" x2="-1.5" y2="3.25" width="0.127" layer="21"/>
<wire x1="-1.5" y1="2.5" x2="4" y2="2.5" width="0.127" layer="21"/>
</package>
<package name="JST-2-PTH-VERT">
<wire x1="-2.95" y1="-2.25" x2="-2.95" y2="2.25" width="0.2032" layer="21"/>
<wire x1="-2.95" y1="2.25" x2="2.95" y2="2.25" width="0.2032" layer="21"/>
<wire x1="2.95" y1="2.25" x2="2.95" y2="-2.25" width="0.2032" layer="21"/>
<wire x1="2.95" y1="-2.25" x2="1" y2="-2.25" width="0.2032" layer="21"/>
<wire x1="-1" y1="-2.25" x2="-2.95" y2="-2.25" width="0.2032" layer="21"/>
<wire x1="-1" y1="-1.75" x2="1" y2="-1.75" width="0.2032" layer="21"/>
<wire x1="1" y1="-1.75" x2="1" y2="-2.25" width="0.2032" layer="21"/>
<wire x1="-1" y1="-1.75" x2="-1" y2="-2.25" width="0.2032" layer="21"/>
<pad name="1" x="-1" y="-0.55" drill="0.7" diameter="1.6256"/>
<pad name="2" x="1" y="-0.55" drill="0.7" diameter="1.6256"/>
<text x="-1.984" y="3" size="0.4064" layer="25">&gt;Name</text>
<text x="2.016" y="3" size="0.4064" layer="27">&gt;Value</text>
<text x="0.616" y="0.75" size="1.27" layer="51">+</text>
<text x="-1.384" y="0.75" size="1.27" layer="51">-</text>
</package>
<package name="1X03">
<wire x1="3.81" y1="0.635" x2="4.445" y2="1.27" width="0.2032" layer="21"/>
<wire x1="4.445" y1="1.27" x2="5.715" y2="1.27" width="0.2032" layer="21"/>
<wire x1="5.715" y1="1.27" x2="6.35" y2="0.635" width="0.2032" layer="21"/>
<wire x1="6.35" y1="-0.635" x2="5.715" y2="-1.27" width="0.2032" layer="21"/>
<wire x1="5.715" y1="-1.27" x2="4.445" y2="-1.27" width="0.2032" layer="21"/>
<wire x1="4.445" y1="-1.27" x2="3.81" y2="-0.635" width="0.2032" layer="21"/>
<wire x1="-0.635" y1="1.27" x2="0.635" y2="1.27" width="0.2032" layer="21"/>
<wire x1="0.635" y1="1.27" x2="1.27" y2="0.635" width="0.2032" layer="21"/>
<wire x1="1.27" y1="-0.635" x2="0.635" y2="-1.27" width="0.2032" layer="21"/>
<wire x1="1.27" y1="0.635" x2="1.905" y2="1.27" width="0.2032" layer="21"/>
<wire x1="1.905" y1="1.27" x2="3.175" y2="1.27" width="0.2032" layer="21"/>
<wire x1="3.175" y1="1.27" x2="3.81" y2="0.635" width="0.2032" layer="21"/>
<wire x1="3.81" y1="-0.635" x2="3.175" y2="-1.27" width="0.2032" layer="21"/>
<wire x1="3.175" y1="-1.27" x2="1.905" y2="-1.27" width="0.2032" layer="21"/>
<wire x1="1.905" y1="-1.27" x2="1.27" y2="-0.635" width="0.2032" layer="21"/>
<wire x1="-1.27" y1="0.635" x2="-1.27" y2="-0.635" width="0.2032" layer="21"/>
<wire x1="-0.635" y1="1.27" x2="-1.27" y2="0.635" width="0.2032" layer="21"/>
<wire x1="-1.27" y1="-0.635" x2="-0.635" y2="-1.27" width="0.2032" layer="21"/>
<wire x1="0.635" y1="-1.27" x2="-0.635" y2="-1.27" width="0.2032" layer="21"/>
<wire x1="6.35" y1="0.635" x2="6.35" y2="-0.635" width="0.2032" layer="21"/>
<pad name="1" x="0" y="0" drill="1.016" diameter="1.8796" rot="R90"/>
<pad name="2" x="2.54" y="0" drill="1.016" diameter="1.8796" rot="R90"/>
<pad name="3" x="5.08" y="0" drill="1.016" diameter="1.8796" rot="R90"/>
<text x="-1.3462" y="1.8288" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.27" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="4.826" y1="-0.254" x2="5.334" y2="0.254" layer="51"/>
<rectangle x1="2.286" y1="-0.254" x2="2.794" y2="0.254" layer="51"/>
<rectangle x1="-0.254" y1="-0.254" x2="0.254" y2="0.254" layer="51"/>
</package>
<package name="MOLEX-1X3">
<wire x1="-1.27" y1="3.048" x2="-1.27" y2="-2.54" width="0.127" layer="21"/>
<wire x1="6.35" y1="3.048" x2="6.35" y2="-2.54" width="0.127" layer="21"/>
<wire x1="6.35" y1="3.048" x2="-1.27" y2="3.048" width="0.127" layer="21"/>
<wire x1="6.35" y1="-2.54" x2="5.08" y2="-2.54" width="0.127" layer="21"/>
<wire x1="5.08" y1="-2.54" x2="0" y2="-2.54" width="0.127" layer="21"/>
<wire x1="0" y1="-2.54" x2="-1.27" y2="-2.54" width="0.127" layer="21"/>
<wire x1="0" y1="-2.54" x2="0" y2="-1.27" width="0.127" layer="21"/>
<wire x1="0" y1="-1.27" x2="5.08" y2="-1.27" width="0.127" layer="21"/>
<wire x1="5.08" y1="-1.27" x2="5.08" y2="-2.54" width="0.127" layer="21"/>
<pad name="1" x="0" y="0" drill="1.016" diameter="1.8796" shape="square"/>
<pad name="2" x="2.54" y="0" drill="1.016" diameter="1.8796"/>
<pad name="3" x="5.08" y="0" drill="1.016" diameter="1.8796"/>
</package>
<package name="SCREWTERMINAL-3.5MM-3">
<wire x1="-2.3" y1="3.4" x2="9.3" y2="3.4" width="0.2032" layer="21"/>
<wire x1="9.3" y1="3.4" x2="9.3" y2="-2.8" width="0.2032" layer="21"/>
<wire x1="9.3" y1="-2.8" x2="9.3" y2="-3.6" width="0.2032" layer="21"/>
<wire x1="9.3" y1="-3.6" x2="-2.3" y2="-3.6" width="0.2032" layer="21"/>
<wire x1="-2.3" y1="-3.6" x2="-2.3" y2="-2.8" width="0.2032" layer="21"/>
<wire x1="-2.3" y1="-2.8" x2="-2.3" y2="3.4" width="0.2032" layer="21"/>
<wire x1="9.3" y1="-2.8" x2="-2.3" y2="-2.8" width="0.2032" layer="21"/>
<wire x1="-2.3" y1="-1.35" x2="-2.7" y2="-1.35" width="0.2032" layer="51"/>
<wire x1="-2.7" y1="-1.35" x2="-2.7" y2="-2.35" width="0.2032" layer="51"/>
<wire x1="-2.7" y1="-2.35" x2="-2.3" y2="-2.35" width="0.2032" layer="51"/>
<wire x1="9.3" y1="3.15" x2="9.7" y2="3.15" width="0.2032" layer="51"/>
<wire x1="9.7" y1="3.15" x2="9.7" y2="2.15" width="0.2032" layer="51"/>
<wire x1="9.7" y1="2.15" x2="9.3" y2="2.15" width="0.2032" layer="51"/>
<pad name="1" x="0" y="0" drill="1.2" diameter="2.413" shape="square"/>
<pad name="2" x="3.5" y="0" drill="1.2" diameter="2.413"/>
<pad name="3" x="7" y="0" drill="1.2" diameter="2.413"/>
<text x="-1.27" y="2.54" size="0.4064" layer="25">&gt;NAME</text>
<text x="-1.27" y="1.27" size="0.4064" layer="27">&gt;VALUE</text>
</package>
<package name="1X03_LOCK">
<wire x1="3.81" y1="0.635" x2="4.445" y2="1.27" width="0.2032" layer="21"/>
<wire x1="4.445" y1="1.27" x2="5.715" y2="1.27" width="0.2032" layer="21"/>
<wire x1="5.715" y1="1.27" x2="6.35" y2="0.635" width="0.2032" layer="21"/>
<wire x1="6.35" y1="-0.635" x2="5.715" y2="-1.27" width="0.2032" layer="21"/>
<wire x1="5.715" y1="-1.27" x2="4.445" y2="-1.27" width="0.2032" layer="21"/>
<wire x1="4.445" y1="-1.27" x2="3.81" y2="-0.635" width="0.2032" layer="21"/>
<wire x1="-0.635" y1="1.27" x2="0.635" y2="1.27" width="0.2032" layer="21"/>
<wire x1="0.635" y1="1.27" x2="1.27" y2="0.635" width="0.2032" layer="21"/>
<wire x1="1.27" y1="-0.635" x2="0.635" y2="-1.27" width="0.2032" layer="21"/>
<wire x1="1.27" y1="0.635" x2="1.905" y2="1.27" width="0.2032" layer="21"/>
<wire x1="1.905" y1="1.27" x2="3.175" y2="1.27" width="0.2032" layer="21"/>
<wire x1="3.175" y1="1.27" x2="3.81" y2="0.635" width="0.2032" layer="21"/>
<wire x1="3.81" y1="-0.635" x2="3.175" y2="-1.27" width="0.2032" layer="21"/>
<wire x1="3.175" y1="-1.27" x2="1.905" y2="-1.27" width="0.2032" layer="21"/>
<wire x1="1.905" y1="-1.27" x2="1.27" y2="-0.635" width="0.2032" layer="21"/>
<wire x1="-1.27" y1="0.635" x2="-1.27" y2="-0.635" width="0.2032" layer="21"/>
<wire x1="-0.635" y1="1.27" x2="-1.27" y2="0.635" width="0.2032" layer="21"/>
<wire x1="-1.27" y1="-0.635" x2="-0.635" y2="-1.27" width="0.2032" layer="21"/>
<wire x1="0.635" y1="-1.27" x2="-0.635" y2="-1.27" width="0.2032" layer="21"/>
<wire x1="6.35" y1="0.635" x2="6.35" y2="-0.635" width="0.2032" layer="21"/>
<pad name="1" x="0" y="0.127" drill="1.016" diameter="1.8796" rot="R90"/>
<pad name="2" x="2.54" y="-0.127" drill="1.016" diameter="1.8796" rot="R90"/>
<pad name="3" x="5.08" y="0.127" drill="1.016" diameter="1.8796" rot="R90"/>
<text x="-1.3462" y="1.8288" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.27" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="4.826" y1="-0.254" x2="5.334" y2="0.254" layer="51"/>
<rectangle x1="2.286" y1="-0.254" x2="2.794" y2="0.254" layer="51"/>
<rectangle x1="-0.254" y1="-0.254" x2="0.254" y2="0.254" layer="51"/>
</package>
<package name="1X03_LOCK_LONGPADS">
<description>This footprint was designed to help hold the alignment of a through-hole component (i.e.  6-pin header) while soldering it into place.  
You may notice that each hole has been shifted either up or down by 0.005 of an inch from it's more standard position (which is a perfectly straight line).  
This slight alteration caused the pins (the squares in the middle) to touch the edges of the holes.  Because they are alternating, it causes a "brace" 
to hold the component in place.  0.005 has proven to be the perfect amount of "off-center" position when using our standard breakaway headers.
Although looks a little odd when you look at the bare footprint, once you have a header in there, the alteration is very hard to notice.  Also,
if you push a header all the way into place, it is covered up entirely on the bottom side.  This idea of altering the position of holes to aid alignment 
will be further integrated into the Sparkfun Library for other footprints.  It can help hold any component with 3 or more connection pins.</description>
<wire x1="1.524" y1="-0.127" x2="1.016" y2="-0.127" width="0.2032" layer="21"/>
<wire x1="4.064" y1="-0.127" x2="3.556" y2="-0.127" width="0.2032" layer="21"/>
<wire x1="-1.27" y1="-0.127" x2="-1.016" y2="-0.127" width="0.2032" layer="21"/>
<wire x1="-1.27" y1="-0.127" x2="-1.27" y2="0.8636" width="0.2032" layer="21"/>
<wire x1="-1.27" y1="0.8636" x2="-0.9906" y2="1.143" width="0.2032" layer="21"/>
<wire x1="-1.27" y1="-0.127" x2="-1.27" y2="-1.1176" width="0.2032" layer="21"/>
<wire x1="-1.27" y1="-1.1176" x2="-0.9906" y2="-1.397" width="0.2032" layer="21"/>
<wire x1="6.35" y1="-0.127" x2="6.096" y2="-0.127" width="0.2032" layer="21"/>
<wire x1="6.35" y1="-0.127" x2="6.35" y2="-1.1176" width="0.2032" layer="21"/>
<wire x1="6.35" y1="-1.1176" x2="6.0706" y2="-1.397" width="0.2032" layer="21"/>
<wire x1="6.35" y1="-0.127" x2="6.35" y2="0.8636" width="0.2032" layer="21"/>
<wire x1="6.35" y1="0.8636" x2="6.0706" y2="1.143" width="0.2032" layer="21"/>
<pad name="1" x="0" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="2" x="2.54" y="-0.254" drill="1.016" shape="long" rot="R90"/>
<pad name="3" x="5.08" y="0" drill="1.016" shape="long" rot="R90"/>
<text x="-1.27" y="1.778" size="1.27" layer="25" font="vector">&gt;NAME</text>
<text x="-1.27" y="-3.302" size="1.27" layer="27" font="vector">&gt;VALUE</text>
<rectangle x1="-0.2921" y1="-0.4191" x2="0.2921" y2="0.1651" layer="51"/>
<rectangle x1="2.2479" y1="-0.4191" x2="2.8321" y2="0.1651" layer="51"/>
<rectangle x1="4.7879" y1="-0.4191" x2="5.3721" y2="0.1651" layer="51"/>
</package>
<package name="MOLEX-1X3_LOCK">
<wire x1="-1.27" y1="3.048" x2="-1.27" y2="-2.54" width="0.127" layer="21"/>
<wire x1="6.35" y1="3.048" x2="6.35" y2="-2.54" width="0.127" layer="21"/>
<wire x1="6.35" y1="3.048" x2="-1.27" y2="3.048" width="0.127" layer="21"/>
<wire x1="6.35" y1="-2.54" x2="5.08" y2="-2.54" width="0.127" layer="21"/>
<wire x1="5.08" y1="-2.54" x2="0" y2="-2.54" width="0.127" layer="21"/>
<wire x1="0" y1="-2.54" x2="-1.27" y2="-2.54" width="0.127" layer="21"/>
<wire x1="0" y1="-2.54" x2="0" y2="-1.27" width="0.127" layer="21"/>
<wire x1="0" y1="-1.27" x2="5.08" y2="-1.27" width="0.127" layer="21"/>
<wire x1="5.08" y1="-1.27" x2="5.08" y2="-2.54" width="0.127" layer="21"/>
<pad name="1" x="0" y="0.127" drill="1.016" diameter="1.8796" shape="square"/>
<pad name="2" x="2.54" y="-0.127" drill="1.016" diameter="1.8796"/>
<pad name="3" x="5.08" y="0.127" drill="1.016" diameter="1.8796"/>
<rectangle x1="-0.2921" y1="-0.2921" x2="0.2921" y2="0.2921" layer="51"/>
<rectangle x1="2.2479" y1="-0.2921" x2="2.8321" y2="0.2921" layer="51"/>
<rectangle x1="4.7879" y1="-0.2921" x2="5.3721" y2="0.2921" layer="51"/>
</package>
<package name="SCREWTERMINAL-3.5MM-3_LOCK.007S">
<wire x1="-2.3" y1="3.4" x2="9.3" y2="3.4" width="0.2032" layer="21"/>
<wire x1="9.3" y1="3.4" x2="9.3" y2="-2.8" width="0.2032" layer="21"/>
<wire x1="9.3" y1="-2.8" x2="9.3" y2="-3.6" width="0.2032" layer="21"/>
<wire x1="9.3" y1="-3.6" x2="-2.3" y2="-3.6" width="0.2032" layer="21"/>
<wire x1="-2.3" y1="-3.6" x2="-2.3" y2="-2.8" width="0.2032" layer="21"/>
<wire x1="-2.3" y1="-2.8" x2="-2.3" y2="3.4" width="0.2032" layer="21"/>
<wire x1="9.3" y1="-2.8" x2="-2.3" y2="-2.8" width="0.2032" layer="21"/>
<wire x1="-2.3" y1="-1.35" x2="-2.7" y2="-1.35" width="0.2032" layer="51"/>
<wire x1="-2.7" y1="-1.35" x2="-2.7" y2="-2.35" width="0.2032" layer="51"/>
<wire x1="-2.7" y1="-2.35" x2="-2.3" y2="-2.35" width="0.2032" layer="51"/>
<wire x1="9.3" y1="3.15" x2="9.7" y2="3.15" width="0.2032" layer="51"/>
<wire x1="9.7" y1="3.15" x2="9.7" y2="2.15" width="0.2032" layer="51"/>
<wire x1="9.7" y1="2.15" x2="9.3" y2="2.15" width="0.2032" layer="51"/>
<circle x="0" y="0" radius="0.425" width="0.001" layer="51"/>
<circle x="3.5" y="0" radius="0.425" width="0.001" layer="51"/>
<circle x="7" y="0" radius="0.425" width="0.001" layer="51"/>
<pad name="1" x="-0.1778" y="0" drill="1.2" diameter="2.032" shape="square"/>
<pad name="2" x="3.5" y="0" drill="1.2" diameter="2.032"/>
<pad name="3" x="7.1778" y="0" drill="1.2" diameter="2.032"/>
<text x="-1.27" y="2.54" size="0.4064" layer="25">&gt;NAME</text>
<text x="-1.27" y="1.27" size="0.4064" layer="27">&gt;VALUE</text>
</package>
<package name="1X03_NO_SILK">
<pad name="1" x="0" y="0" drill="1.016" diameter="1.8796" rot="R90"/>
<pad name="2" x="2.54" y="0" drill="1.016" diameter="1.8796" rot="R90"/>
<pad name="3" x="5.08" y="0" drill="1.016" diameter="1.8796" rot="R90"/>
<text x="-1.3462" y="1.8288" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.27" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="4.826" y1="-0.254" x2="5.334" y2="0.254" layer="51"/>
<rectangle x1="2.286" y1="-0.254" x2="2.794" y2="0.254" layer="51"/>
<rectangle x1="-0.254" y1="-0.254" x2="0.254" y2="0.254" layer="51"/>
</package>
<package name="1X03_LONGPADS">
<wire x1="-1.27" y1="0.635" x2="-1.27" y2="-0.635" width="0.2032" layer="21"/>
<wire x1="6.35" y1="0.635" x2="6.35" y2="-0.635" width="0.2032" layer="21"/>
<pad name="1" x="0" y="0" drill="1.1176" diameter="1.8796" shape="long" rot="R90"/>
<pad name="2" x="2.54" y="0" drill="1.1176" diameter="1.8796" shape="long" rot="R90"/>
<pad name="3" x="5.08" y="0" drill="1.1176" diameter="1.8796" shape="long" rot="R90"/>
<text x="-1.3462" y="2.4638" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.27" y="-3.81" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="4.826" y1="-0.254" x2="5.334" y2="0.254" layer="51"/>
<rectangle x1="2.286" y1="-0.254" x2="2.794" y2="0.254" layer="51"/>
<rectangle x1="-0.254" y1="-0.254" x2="0.254" y2="0.254" layer="51"/>
</package>
<package name="JST-3-PTH">
<wire x1="-3.95" y1="-1.6" x2="-3.95" y2="6" width="0.2032" layer="21"/>
<wire x1="-3.95" y1="6" x2="3.95" y2="6" width="0.2032" layer="21"/>
<wire x1="3.95" y1="6" x2="3.95" y2="-1.6" width="0.2032" layer="21"/>
<wire x1="-3.95" y1="-1.6" x2="-3.3" y2="-1.6" width="0.2032" layer="21"/>
<wire x1="3.95" y1="-1.6" x2="3.3" y2="-1.6" width="0.2032" layer="21"/>
<wire x1="-3.3" y1="-1.6" x2="-3.3" y2="0" width="0.2032" layer="21"/>
<wire x1="3.3" y1="-1.6" x2="3.3" y2="0" width="0.2032" layer="21"/>
<pad name="1" x="-2" y="0" drill="0.7" diameter="1.6256"/>
<pad name="2" x="0" y="0" drill="0.7" diameter="1.6256"/>
<pad name="3" x="2" y="0" drill="0.7" diameter="1.6256"/>
<text x="-1.27" y="5.24" size="0.4064" layer="25">&gt;Name</text>
<text x="-1.27" y="3.97" size="0.4064" layer="27">&gt;Value</text>
<text x="-2.4" y="0.67" size="1.27" layer="51">+</text>
<text x="-0.4" y="0.67" size="1.27" layer="51">-</text>
<text x="1.7" y="0.87" size="0.8" layer="51">S</text>
</package>
<package name="1X03_PP_HOLES_ONLY">
<circle x="0" y="0" radius="0.635" width="0.127" layer="51"/>
<circle x="2.54" y="0" radius="0.635" width="0.127" layer="51"/>
<circle x="5.08" y="0" radius="0.635" width="0.127" layer="51"/>
<pad name="1" x="0" y="0" drill="0.9" diameter="0.8128" rot="R90"/>
<pad name="2" x="2.54" y="0" drill="0.9" diameter="0.8128" rot="R90"/>
<pad name="3" x="5.08" y="0" drill="0.9" diameter="0.8128" rot="R90"/>
<hole x="0" y="0" drill="1.4732"/>
<hole x="2.54" y="0" drill="1.4732"/>
<hole x="5.08" y="0" drill="1.4732"/>
</package>
<package name="SCREWTERMINAL-5MM-3">
<wire x1="-3.1" y1="4.2" x2="13.1" y2="4.2" width="0.2032" layer="21"/>
<wire x1="13.1" y1="4.2" x2="13.1" y2="-2.3" width="0.2032" layer="21"/>
<wire x1="13.1" y1="-2.3" x2="13.1" y2="-3.3" width="0.2032" layer="21"/>
<wire x1="13.1" y1="-3.3" x2="-3.1" y2="-3.3" width="0.2032" layer="21"/>
<wire x1="-3.1" y1="-3.3" x2="-3.1" y2="-2.3" width="0.2032" layer="21"/>
<wire x1="-3.1" y1="-2.3" x2="-3.1" y2="4.2" width="0.2032" layer="21"/>
<wire x1="13.1" y1="-2.3" x2="-3.1" y2="-2.3" width="0.2032" layer="21"/>
<wire x1="-3.1" y1="-1.35" x2="-3.7" y2="-1.35" width="0.2032" layer="51"/>
<wire x1="-3.7" y1="-1.35" x2="-3.7" y2="-2.35" width="0.2032" layer="51"/>
<wire x1="-3.7" y1="-2.35" x2="-3.1" y2="-2.35" width="0.2032" layer="51"/>
<wire x1="13.1" y1="4" x2="13.7" y2="4" width="0.2032" layer="51"/>
<wire x1="13.7" y1="4" x2="13.7" y2="3" width="0.2032" layer="51"/>
<wire x1="13.7" y1="3" x2="13.1" y2="3" width="0.2032" layer="51"/>
<circle x="2.5" y="3.7" radius="0.2828" width="0.127" layer="51"/>
<pad name="1" x="0" y="0" drill="1.3" diameter="2.413" shape="square"/>
<pad name="2" x="5" y="0" drill="1.3" diameter="2.413"/>
<pad name="3" x="10" y="0" drill="1.3" diameter="2.413"/>
<text x="-1.27" y="2.54" size="0.4064" layer="25">&gt;NAME</text>
<text x="-1.27" y="1.27" size="0.4064" layer="27">&gt;VALUE</text>
</package>
<package name="1X03_LOCK_NO_SILK">
<pad name="1" x="0" y="0.127" drill="1.016" diameter="1.8796" rot="R90"/>
<pad name="2" x="2.54" y="-0.127" drill="1.016" diameter="1.8796" rot="R90"/>
<pad name="3" x="5.08" y="0.127" drill="1.016" diameter="1.8796" rot="R90"/>
<text x="-1.3462" y="1.8288" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.27" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="4.826" y1="-0.254" x2="5.334" y2="0.254" layer="51"/>
<rectangle x1="2.286" y1="-0.254" x2="2.794" y2="0.254" layer="51"/>
<rectangle x1="-0.254" y1="-0.254" x2="0.254" y2="0.254" layer="51"/>
</package>
<package name="JST-3-SMD">
<wire x1="-4.99" y1="-2.07" x2="-4.99" y2="-5.57" width="0.2032" layer="21"/>
<wire x1="-4.99" y1="-5.57" x2="-4.19" y2="-5.57" width="0.2032" layer="21"/>
<wire x1="-4.19" y1="-5.57" x2="-4.19" y2="-3.07" width="0.2032" layer="21"/>
<wire x1="-4.19" y1="-3.07" x2="-2.99" y2="-3.07" width="0.2032" layer="21"/>
<wire x1="3.01" y1="-3.07" x2="4.21" y2="-3.07" width="0.2032" layer="21"/>
<wire x1="4.21" y1="-3.07" x2="4.21" y2="-5.57" width="0.2032" layer="21"/>
<wire x1="4.21" y1="-5.57" x2="5.01" y2="-5.57" width="0.2032" layer="21"/>
<wire x1="5.01" y1="-5.57" x2="5.01" y2="-2.07" width="0.2032" layer="21"/>
<wire x1="3.01" y1="1.93" x2="-2.99" y2="1.93" width="0.2032" layer="21"/>
<smd name="1" x="-1.99" y="-4.77" dx="1" dy="4.6" layer="1"/>
<smd name="3" x="2.01" y="-4.77" dx="1" dy="4.6" layer="1"/>
<smd name="NC1" x="-4.39" y="0.43" dx="3.4" dy="1.6" layer="1" rot="R90"/>
<smd name="NC2" x="4.41" y="0.43" dx="3.4" dy="1.6" layer="1" rot="R90"/>
<smd name="2" x="0.01" y="-4.77" dx="1" dy="4.6" layer="1"/>
<text x="-2.26" y="0.2" size="0.4064" layer="25">&gt;Name</text>
<text x="-2.26" y="-1.07" size="0.4064" layer="27">&gt;Value</text>
</package>
<package name="1X03-1MM-RA">
<wire x1="-1" y1="-4.6" x2="1" y2="-4.6" width="0.254" layer="21"/>
<wire x1="-2.5" y1="-2" x2="-2.5" y2="-0.35" width="0.254" layer="21"/>
<wire x1="1.75" y1="-0.35" x2="2.4997" y2="-0.35" width="0.254" layer="21"/>
<wire x1="2.4997" y1="-0.35" x2="2.4997" y2="-2" width="0.254" layer="21"/>
<wire x1="-2.5" y1="-0.35" x2="-1.75" y2="-0.35" width="0.254" layer="21"/>
<circle x="-2" y="0.3" radius="0.1414" width="0.4" layer="21"/>
<smd name="NC2" x="-2.3" y="-3.675" dx="1.2" dy="2" layer="1"/>
<smd name="NC1" x="2.3" y="-3.675" dx="1.2" dy="2" layer="1"/>
<smd name="1" x="-1" y="0" dx="0.6" dy="1.35" layer="1"/>
<smd name="2" x="0" y="0" dx="0.6" dy="1.35" layer="1"/>
<smd name="3" x="1" y="0" dx="0.6" dy="1.35" layer="1"/>
<text x="-1.73" y="1.73" size="0.4064" layer="25" rot="R180">&gt;NAME</text>
<text x="3.46" y="1.73" size="0.4064" layer="27" rot="R180">&gt;VALUE</text>
</package>
<package name="1X03_SMD_RA_FEMALE">
<wire x1="-3.935" y1="4.25" x2="-3.935" y2="-4.25" width="0.1778" layer="21"/>
<wire x1="3.935" y1="4.25" x2="-3.935" y2="4.25" width="0.1778" layer="21"/>
<wire x1="3.935" y1="-4.25" x2="3.935" y2="4.25" width="0.1778" layer="21"/>
<wire x1="-3.935" y1="-4.25" x2="3.935" y2="-4.25" width="0.1778" layer="21"/>
<rectangle x1="-0.32" y1="6.8" x2="0.32" y2="7.65" layer="51"/>
<rectangle x1="2.22" y1="6.8" x2="2.86" y2="7.65" layer="51"/>
<rectangle x1="-2.86" y1="6.8" x2="-2.22" y2="7.65" layer="51"/>
<smd name="3" x="2.54" y="7.225" dx="1.25" dy="3" layer="1" rot="R180"/>
<smd name="2" x="0" y="7.225" dx="1.25" dy="3" layer="1" rot="R180"/>
<smd name="1" x="-2.54" y="7.225" dx="1.25" dy="3" layer="1" rot="R180"/>
<text x="-3.155" y="2.775" size="1" layer="27">&gt;Value</text>
<text x="-2.955" y="-3.395" size="1" layer="25">&gt;Name</text>
</package>
<package name="1X03_SMD_RA_MALE">
<wire x1="3.81" y1="1.25" x2="-3.81" y2="1.25" width="0.1778" layer="51"/>
<wire x1="-3.81" y1="1.25" x2="-3.81" y2="-1.25" width="0.1778" layer="51"/>
<wire x1="3.81" y1="-1.25" x2="2.53" y2="-1.25" width="0.1778" layer="51"/>
<wire x1="2.53" y1="-1.25" x2="-0.01" y2="-1.25" width="0.1778" layer="51"/>
<wire x1="-0.01" y1="-1.25" x2="-2.55" y2="-1.25" width="0.1778" layer="51"/>
<wire x1="-2.55" y1="-1.25" x2="-3.81" y2="-1.25" width="0.1778" layer="51"/>
<wire x1="3.81" y1="-1.25" x2="3.81" y2="1.25" width="0.1778" layer="51"/>
<wire x1="2.53" y1="-1.25" x2="2.53" y2="-7.25" width="0.127" layer="51"/>
<wire x1="-0.01" y1="-1.25" x2="-0.01" y2="-7.25" width="0.127" layer="51"/>
<wire x1="-2.55" y1="-1.25" x2="-2.55" y2="-7.25" width="0.127" layer="51"/>
<rectangle x1="-0.32" y1="4.15" x2="0.32" y2="5.95" layer="51"/>
<rectangle x1="-2.86" y1="4.15" x2="-2.22" y2="5.95" layer="51"/>
<rectangle x1="2.22" y1="4.15" x2="2.86" y2="5.95" layer="51"/>
<smd name="1" x="-2.54" y="5" dx="3" dy="1" layer="1" rot="R90"/>
<smd name="2" x="0" y="5" dx="3" dy="1" layer="1" rot="R90"/>
<smd name="3" x="2.54" y="5" dx="3" dy="1" layer="1" rot="R90"/>
</package>
<package name="1X03_SMD_RA_MALE_POST">
<description>&lt;h3&gt;SMD 3-Pin Male Right-Angle Header w/ Alignment posts&lt;/h3&gt;

Matches 4UCONN part # 11026&lt;br&gt;
&lt;a href="http://www.4uconnector.com/online/object/4udrawing/11026.pdf"&gt;http://www.4uconnector.com/online/object/4udrawing/11026.pdf&lt;/a&gt;</description>
<wire x1="3.81" y1="1.25" x2="-3.81" y2="1.25" width="0.1778" layer="51"/>
<wire x1="-3.81" y1="1.25" x2="-3.81" y2="-1.25" width="0.1778" layer="51"/>
<wire x1="3.81" y1="-1.25" x2="2.53" y2="-1.25" width="0.1778" layer="51"/>
<wire x1="2.53" y1="-1.25" x2="-0.01" y2="-1.25" width="0.1778" layer="51"/>
<wire x1="-0.01" y1="-1.25" x2="-2.55" y2="-1.25" width="0.1778" layer="51"/>
<wire x1="-2.55" y1="-1.25" x2="-3.81" y2="-1.25" width="0.1778" layer="51"/>
<wire x1="3.81" y1="-1.25" x2="3.81" y2="1.25" width="0.1778" layer="51"/>
<wire x1="2.53" y1="-1.25" x2="2.53" y2="-7.25" width="0.127" layer="51"/>
<wire x1="-0.01" y1="-1.25" x2="-0.01" y2="-7.25" width="0.127" layer="51"/>
<wire x1="-2.55" y1="-1.25" x2="-2.55" y2="-7.25" width="0.127" layer="51"/>
<rectangle x1="-0.32" y1="4.15" x2="0.32" y2="5.95" layer="51"/>
<rectangle x1="-2.86" y1="4.15" x2="-2.22" y2="5.95" layer="51"/>
<rectangle x1="2.22" y1="4.15" x2="2.86" y2="5.95" layer="51"/>
<smd name="1" x="-2.54" y="5.07" dx="2.5" dy="1.27" layer="1" rot="R90"/>
<smd name="2" x="0" y="5.07" dx="2.5" dy="1.27" layer="1" rot="R90"/>
<smd name="3" x="2.54" y="5.07" dx="2.5" dy="1.27" layer="1" rot="R90"/>
<hole x="-1.27" y="0" drill="1.6"/>
<hole x="1.27" y="0" drill="1.6"/>
</package>
<package name="JST-3-PTH-VERT">
<description>This 3-pin connector mates with the JST cable sold on SparkFun.</description>
<wire x1="-3.95" y1="-2.25" x2="-3.95" y2="2.25" width="0.2032" layer="21"/>
<wire x1="-3.95" y1="2.25" x2="3.95" y2="2.25" width="0.2032" layer="21"/>
<wire x1="3.95" y1="2.25" x2="3.95" y2="-2.25" width="0.2032" layer="21"/>
<wire x1="3.95" y1="-2.25" x2="1" y2="-2.25" width="0.2032" layer="21"/>
<wire x1="-1" y1="-2.25" x2="-3.95" y2="-2.25" width="0.2032" layer="21"/>
<wire x1="-1" y1="-1.75" x2="1" y2="-1.75" width="0.2032" layer="21"/>
<wire x1="1" y1="-1.75" x2="1" y2="-2.25" width="0.2032" layer="21"/>
<wire x1="-1" y1="-1.75" x2="-1" y2="-2.25" width="0.2032" layer="21"/>
<pad name="1" x="-2" y="-0.55" drill="0.7" diameter="1.6256"/>
<pad name="2" x="0" y="-0.55" drill="0.7" diameter="1.6256"/>
<pad name="3" x="2" y="-0.55" drill="0.7" diameter="1.6256"/>
<text x="-3" y="3" size="0.4064" layer="25">&gt;Name</text>
<text x="1" y="3" size="0.4064" layer="27">&gt;Value</text>
<text x="-2.4" y="0.75" size="1.27" layer="51">+</text>
<text x="-0.4" y="0.75" size="1.27" layer="51">-</text>
<text x="1.7" y="0.95" size="0.8" layer="51">S</text>
</package>
<package name="1X03_SMD_RA_MALE_POST_SMALLER">
<wire x1="3.81" y1="1.25" x2="-3.81" y2="1.25" width="0.1778" layer="51"/>
<wire x1="-3.81" y1="1.25" x2="-3.81" y2="-1.25" width="0.1778" layer="51"/>
<wire x1="3.81" y1="-1.25" x2="2.53" y2="-1.25" width="0.1778" layer="51"/>
<wire x1="2.53" y1="-1.25" x2="-0.01" y2="-1.25" width="0.1778" layer="51"/>
<wire x1="-0.01" y1="-1.25" x2="-2.55" y2="-1.25" width="0.1778" layer="51"/>
<wire x1="-2.55" y1="-1.25" x2="-3.81" y2="-1.25" width="0.1778" layer="51"/>
<wire x1="3.81" y1="-1.25" x2="3.81" y2="1.25" width="0.1778" layer="51"/>
<wire x1="2.53" y1="-1.25" x2="2.53" y2="-7.25" width="0.127" layer="51"/>
<wire x1="-0.01" y1="-1.25" x2="-0.01" y2="-7.25" width="0.127" layer="51"/>
<wire x1="-2.55" y1="-1.25" x2="-2.55" y2="-7.25" width="0.127" layer="51"/>
<rectangle x1="-0.32" y1="4.15" x2="0.32" y2="5.95" layer="51"/>
<rectangle x1="-2.86" y1="4.15" x2="-2.22" y2="5.95" layer="51"/>
<rectangle x1="2.22" y1="4.15" x2="2.86" y2="5.95" layer="51"/>
<smd name="1" x="-2.54" y="5.07" dx="2.5" dy="1.27" layer="1" rot="R90"/>
<smd name="2" x="0" y="5.07" dx="2.5" dy="1.27" layer="1" rot="R90"/>
<smd name="3" x="2.54" y="5.07" dx="2.5" dy="1.27" layer="1" rot="R90"/>
<hole x="-1.27" y="0" drill="1.397"/>
<hole x="1.27" y="0" drill="1.397"/>
</package>
<package name="1X03_SMD_RA_MALE_POST_SMALLEST">
<wire x1="3.81" y1="1.25" x2="-3.81" y2="1.25" width="0.1778" layer="51"/>
<wire x1="-3.81" y1="1.25" x2="-3.81" y2="-1.25" width="0.1778" layer="51"/>
<wire x1="3.81" y1="-1.25" x2="2.53" y2="-1.25" width="0.1778" layer="51"/>
<wire x1="2.53" y1="-1.25" x2="-0.01" y2="-1.25" width="0.1778" layer="51"/>
<wire x1="-0.01" y1="-1.25" x2="-2.55" y2="-1.25" width="0.1778" layer="51"/>
<wire x1="-2.55" y1="-1.25" x2="-3.81" y2="-1.25" width="0.1778" layer="51"/>
<wire x1="3.81" y1="-1.25" x2="3.81" y2="1.25" width="0.1778" layer="51"/>
<wire x1="2.53" y1="-1.25" x2="2.53" y2="-7.25" width="0.127" layer="51"/>
<wire x1="-0.01" y1="-1.25" x2="-0.01" y2="-7.25" width="0.127" layer="51"/>
<wire x1="-2.55" y1="-1.25" x2="-2.55" y2="-7.25" width="0.127" layer="51"/>
<rectangle x1="-0.32" y1="4.15" x2="0.32" y2="5.95" layer="51"/>
<rectangle x1="-2.86" y1="4.15" x2="-2.22" y2="5.95" layer="51"/>
<rectangle x1="2.22" y1="4.15" x2="2.86" y2="5.95" layer="51"/>
<smd name="1" x="-2.54" y="5.07" dx="2.5" dy="1.27" layer="1" rot="R90"/>
<smd name="2" x="0" y="5.07" dx="2.5" dy="1.27" layer="1" rot="R90"/>
<smd name="3" x="2.54" y="5.07" dx="2.5" dy="1.27" layer="1" rot="R90"/>
<hole x="-1.27" y="0" drill="1.3208"/>
<hole x="1.27" y="0" drill="1.3208"/>
</package>
<package name="1X03_2.54_SCREWTERM">
<pad name="P2" x="0" y="0" drill="1.016" shape="square"/>
<pad name="P1" x="2.54" y="0" drill="1.016" shape="square"/>
<wire x1="-4.04" y1="3.25" x2="4" y2="3.25" width="0.127" layer="21"/>
<wire x1="4" y1="3.25" x2="4" y2="2.5" width="0.127" layer="21"/>
<wire x1="4" y1="2.5" x2="4" y2="-3.25" width="0.127" layer="21"/>
<wire x1="4" y1="-3.25" x2="-4.04" y2="-3.25" width="0.127" layer="21"/>
<wire x1="-4.04" y1="-3.25" x2="-4.04" y2="2.5" width="0.127" layer="21"/>
<wire x1="-4.04" y1="2.5" x2="-4.04" y2="3.25" width="0.127" layer="21"/>
<wire x1="-4.04" y1="2.5" x2="4" y2="2.5" width="0.127" layer="21"/>
<pad name="P3" x="-2.54" y="0" drill="1.016" shape="square"/>
</package>
<package name="MA03-2CONTACT">
<wire x1="-3.175" y1="2.54" x2="-1.905" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="2.54" x2="-1.27" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="1.905" x2="-0.635" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="2.54" x2="0.635" y2="2.54" width="0.1524" layer="21"/>
<wire x1="0.635" y1="2.54" x2="1.27" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="2.54" x2="-3.81" y2="1.905" width="0.1524" layer="21"/>
<wire x1="1.27" y1="1.905" x2="1.905" y2="2.54" width="0.1524" layer="21"/>
<wire x1="1.905" y1="2.54" x2="3.175" y2="2.54" width="0.1524" layer="21"/>
<wire x1="3.175" y1="2.54" x2="3.81" y2="1.905" width="0.1524" layer="21"/>
<wire x1="3.81" y1="1.905" x2="3.81" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="-1.905" x2="-1.905" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="1.27" y1="-1.905" x2="0.635" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="0.635" y1="-2.54" x2="-0.635" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="-2.54" x2="-1.27" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-3.81" y1="1.905" x2="-3.81" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-3.81" y1="-1.905" x2="-3.175" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="-2.54" x2="-3.175" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="3.81" y1="-1.905" x2="3.175" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-2.54" x2="1.905" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="1.905" y1="-2.54" x2="1.27" y2="-1.905" width="0.1524" layer="21"/>
<pad name="1" x="-2.54" y="-1.27" drill="0.7" diameter="1.6764"/>
<pad name="3" x="0" y="-1.27" drill="0.7" diameter="1.6764" shape="octagon"/>
<pad name="5" x="2.54" y="-1.27" drill="0.7" diameter="1.6764" shape="octagon"/>
<pad name="2" x="-2.54" y="1.27" drill="0.7" diameter="1.6764" shape="octagon"/>
<pad name="4" x="0" y="1.27" drill="0.7" diameter="1.6764" shape="octagon"/>
<pad name="6" x="2.54" y="1.27" drill="0.7" diameter="1.6764" shape="octagon"/>
<text x="-3.175" y="-3.556" size="0.8128" layer="21" ratio="10">1</text>
<text x="-3.81" y="2.921" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.27" y="-4.191" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-0.254" y1="-1.524" x2="0.254" y2="-1.016" layer="51"/>
<rectangle x1="-2.794" y1="-1.524" x2="-2.286" y2="-1.016" layer="51"/>
<rectangle x1="2.286" y1="-1.524" x2="2.794" y2="-1.016" layer="51"/>
<rectangle x1="-2.794" y1="1.016" x2="-2.286" y2="1.524" layer="51"/>
<rectangle x1="-0.254" y1="1.016" x2="0.254" y2="1.524" layer="51"/>
<rectangle x1="2.286" y1="1.016" x2="2.794" y2="1.524" layer="51"/>
</package>
<package name="MCP6001_SC70-5">
<smd name="1" x="-1.05" y="0.65" dx="1.05" dy="0.4" layer="1"/>
<smd name="2" x="-1.05" y="0" dx="1.05" dy="0.4" layer="1"/>
<smd name="3" x="-1.05" y="-0.65" dx="1.05" dy="0.4" layer="1"/>
<smd name="4" x="1.05" y="-0.65" dx="1.05" dy="0.4" layer="1"/>
<smd name="5" x="1.05" y="0.65" dx="1.05" dy="0.4" layer="1"/>
<text x="0" y="2.54" size="1.27" layer="25" align="center">&gt;NAME</text>
<text x="0" y="-2.54" size="1.27" layer="27" align="center">&gt;VALUE</text>
<wire x1="-1.825" y1="1.35" x2="1.825" y2="1.35" width="0.05" layer="51"/>
<wire x1="1.825" y1="1.35" x2="1.825" y2="-1.35" width="0.05" layer="51"/>
<wire x1="1.825" y1="-1.35" x2="-1.825" y2="-1.35" width="0.05" layer="51"/>
<wire x1="-1.825" y1="-1.35" x2="-1.825" y2="1.35" width="0.05" layer="51"/>
<wire x1="-0.625" y1="1" x2="0.625" y2="1" width="0.1" layer="51"/>
<wire x1="0.625" y1="1" x2="0.625" y2="-1" width="0.1" layer="51"/>
<wire x1="0.625" y1="-1" x2="-0.625" y2="-1" width="0.1" layer="51"/>
<wire x1="-0.625" y1="-1" x2="-0.625" y2="1" width="0.1" layer="51"/>
<wire x1="-0.625" y1="0.35" x2="0.025" y2="1" width="0.1" layer="51"/>
<wire x1="-0.175" y1="1" x2="0.175" y2="1" width="0.2" layer="21"/>
<wire x1="0.175" y1="1" x2="0.175" y2="-1" width="0.2" layer="21"/>
<wire x1="0.175" y1="-1" x2="-0.175" y2="-1" width="0.2" layer="21"/>
<wire x1="-0.175" y1="-1" x2="-0.175" y2="1" width="0.2" layer="21"/>
<wire x1="-1.575" y1="1.1" x2="-0.525" y2="1.1" width="0.2" layer="21"/>
</package>
<package name="SOT23-5">
<wire x1="-1.973" y1="1.983" x2="1.973" y2="1.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="-1.983" x2="-1.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="-1.973" y1="-1.983" x2="-1.973" y2="1.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="1.983" x2="1.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="1.422" y1="0.66" x2="1.422" y2="-0.66" width="0.1524" layer="51"/>
<wire x1="1.422" y1="-0.66" x2="-1.422" y2="-0.66" width="0.1524" layer="51"/>
<wire x1="-1.422" y1="-0.66" x2="-1.422" y2="0.66" width="0.1524" layer="51"/>
<wire x1="-1.422" y1="0.66" x2="1.422" y2="0.66" width="0.1524" layer="51"/>
<smd name="5" x="-0.95" y="1.1" dx="0.4" dy="1" layer="1"/>
<smd name="3" x="0.95" y="-1.1" dx="0.4" dy="1" layer="1"/>
<smd name="1" x="-0.95" y="-1.1" dx="0.4" dy="1" layer="1"/>
<smd name="2" x="0" y="-1.1" dx="0.4" dy="1" layer="1"/>
<smd name="4" x="0.95" y="1.1" dx="0.4" dy="1" layer="1"/>
<text x="-1.905" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.1786" y1="0.7112" x2="-0.7214" y2="1.2954" layer="51"/>
<rectangle x1="0.7112" y1="-1.2954" x2="1.1684" y2="-0.7112" layer="51"/>
<rectangle x1="-1.1684" y1="-1.2954" x2="-0.7112" y2="-0.7112" layer="51"/>
<rectangle x1="-0.2184" y1="-1.2954" x2="0.2388" y2="-0.7112" layer="51"/>
<rectangle x1="0.7214" y1="0.7112" x2="1.1786" y2="1.2954" layer="51"/>
</package>
<package name="SO16NBC">
<description>MLX75306 image sensor package.  Line indicate optical sensor</description>
<wire x1="4.674671875" y1="1.642103125" x2="-4.723328125" y2="1.642103125" width="0.1524" layer="21"/>
<wire x1="4.674671875" y1="-2.269496875" x2="5.055671875" y2="-1.888496875" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.104328125" y1="1.261103125" x2="-4.723328125" y2="1.642103125" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.674671875" y1="1.642103125" x2="5.055671875" y2="1.261103125" width="0.1524" layer="21" curve="-90"/>
<wire x1="-5.104328125" y1="-1.888496875" x2="-4.723328125" y2="-2.269496875" width="0.1524" layer="21" curve="90"/>
<wire x1="-4.723328125" y1="-2.269496875" x2="4.674671875" y2="-2.269496875" width="0.1524" layer="21"/>
<wire x1="5.055671875" y1="-1.888496875" x2="5.055671875" y2="1.261103125" width="0.1524" layer="21"/>
<wire x1="-5.104328125" y1="1.261103125" x2="-5.104328125" y2="-1.888496875" width="0.1524" layer="21"/>
<wire x1="-5.104328125" y1="0.194303125" x2="-5.104328125" y2="-0.821696875" width="0.1524" layer="21" curve="-180"/>
<wire x1="-5.104328125" y1="-1.913896875" x2="5.055671875" y2="-1.913896875" width="0.0508" layer="21"/>
<smd name="1" x="-4.469328125" y="-3.387096875" dx="0.6604" dy="2.032" layer="1"/>
<smd name="16" x="-4.469328125" y="2.759703125" dx="0.6604" dy="2.032" layer="1"/>
<smd name="2" x="-3.199328125" y="-3.387096875" dx="0.6604" dy="2.032" layer="1"/>
<smd name="3" x="-1.929328125" y="-3.387096875" dx="0.6604" dy="2.032" layer="1"/>
<smd name="15" x="-3.199328125" y="2.759703125" dx="0.6604" dy="2.032" layer="1"/>
<smd name="14" x="-1.929328125" y="2.759703125" dx="0.6604" dy="2.032" layer="1"/>
<smd name="4" x="-0.659328125" y="-3.387096875" dx="0.6604" dy="2.032" layer="1"/>
<smd name="13" x="-0.659328125" y="2.759703125" dx="0.6604" dy="2.032" layer="1"/>
<smd name="5" x="0.610671875" y="-3.387096875" dx="0.6604" dy="2.032" layer="1"/>
<smd name="12" x="0.610671875" y="2.759703125" dx="0.6604" dy="2.032" layer="1"/>
<smd name="6" x="1.880671875" y="-3.387096875" dx="0.6604" dy="2.032" layer="1"/>
<smd name="7" x="3.150671875" y="-3.387096875" dx="0.6604" dy="2.032" layer="1"/>
<smd name="11" x="1.880671875" y="2.759703125" dx="0.6604" dy="2.032" layer="1"/>
<smd name="10" x="3.150671875" y="2.759703125" dx="0.6604" dy="2.032" layer="1"/>
<smd name="8" x="4.420671875" y="-3.387096875" dx="0.6604" dy="2.032" layer="1"/>
<smd name="9" x="4.420671875" y="2.759703125" dx="0.6604" dy="2.032" layer="1"/>
<text x="6.411671875" y="0.551303125" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="-5.485328125" y="-2.345696875" size="1.27" layer="25" ratio="10" rot="R90">&gt;NAME</text>
<rectangle x1="-0.913328125" y1="1.642103125" x2="-0.405328125" y2="2.785103125" layer="51"/>
<rectangle x1="-4.723328125" y1="-3.412496875" x2="-4.215328125" y2="-2.269496875" layer="51"/>
<rectangle x1="-3.453328125" y1="-3.412496875" x2="-2.945328125" y2="-2.269496875" layer="51"/>
<rectangle x1="-2.183328125" y1="-3.387096875" x2="-1.675328125" y2="-2.244096875" layer="51"/>
<rectangle x1="-0.913328125" y1="-3.412496875" x2="-0.405328125" y2="-2.269496875" layer="51"/>
<rectangle x1="-2.183328125" y1="1.642103125" x2="-1.675328125" y2="2.785103125" layer="51"/>
<rectangle x1="-3.453328125" y1="1.642103125" x2="-2.945328125" y2="2.785103125" layer="51"/>
<rectangle x1="-4.723328125" y1="1.642103125" x2="-4.215328125" y2="2.785103125" layer="51"/>
<rectangle x1="0.356671875" y1="-3.412496875" x2="0.864671875" y2="-2.269496875" layer="51"/>
<rectangle x1="1.626671875" y1="-3.412496875" x2="2.134671875" y2="-2.269496875" layer="51"/>
<rectangle x1="2.896671875" y1="-3.412496875" x2="3.404671875" y2="-2.269496875" layer="51"/>
<rectangle x1="4.166671875" y1="-3.412496875" x2="4.674671875" y2="-2.269496875" layer="51"/>
<rectangle x1="0.356671875" y1="1.642103125" x2="0.864671875" y2="2.785103125" layer="51"/>
<rectangle x1="1.626671875" y1="1.642103125" x2="2.134671875" y2="2.785103125" layer="51"/>
<rectangle x1="2.896671875" y1="1.642103125" x2="3.404671875" y2="2.785103125" layer="51"/>
<rectangle x1="4.166671875" y1="1.642103125" x2="4.674671875" y2="2.785103125" layer="51"/>
<wire x1="3.524171875" y1="-0.006696875" x2="-3.572828125" y2="-0.006696875" width="0.1524" layer="21"/>
<wire x1="-0.024328125" y1="0.993303125" x2="-0.024328125" y2="-1.006696875" width="0.1524" layer="21"/>
</package>
<package name="MA03-2SMT">
<wire x1="-3.175" y1="2.54" x2="-1.905" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="2.54" x2="-1.27" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="1.905" x2="-0.635" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="2.54" x2="0.635" y2="2.54" width="0.1524" layer="21"/>
<wire x1="0.635" y1="2.54" x2="1.27" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="2.54" x2="-3.81" y2="1.905" width="0.1524" layer="21"/>
<wire x1="1.27" y1="1.905" x2="1.905" y2="2.54" width="0.1524" layer="21"/>
<wire x1="1.905" y1="2.54" x2="3.175" y2="2.54" width="0.1524" layer="21"/>
<wire x1="3.175" y1="2.54" x2="3.81" y2="1.905" width="0.1524" layer="21"/>
<wire x1="3.81" y1="1.905" x2="3.81" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="-1.905" x2="-1.905" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="1.27" y1="-1.905" x2="0.635" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="0.635" y1="-2.54" x2="-0.635" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="-2.54" x2="-1.27" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-3.81" y1="1.905" x2="-3.81" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-3.81" y1="-1.905" x2="-3.175" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="-2.54" x2="-3.175" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="3.81" y1="-1.905" x2="3.175" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-2.54" x2="1.905" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="1.905" y1="-2.54" x2="1.27" y2="-1.905" width="0.1524" layer="21"/>
<text x="-3.175" y="-3.556" size="0.8128" layer="21" ratio="10">1</text>
<text x="-3.81" y="2.921" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.27" y="-4.191" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-0.254" y1="-1.524" x2="0.254" y2="-1.016" layer="51"/>
<rectangle x1="-2.794" y1="-1.524" x2="-2.286" y2="-1.016" layer="51"/>
<rectangle x1="2.286" y1="-1.524" x2="2.794" y2="-1.016" layer="51"/>
<rectangle x1="-2.794" y1="1.016" x2="-2.286" y2="1.524" layer="51"/>
<rectangle x1="-0.254" y1="1.016" x2="0.254" y2="1.524" layer="51"/>
<rectangle x1="2.286" y1="1.016" x2="2.794" y2="1.524" layer="51"/>
<smd name="P$1" x="-2.54" y="-1.27" dx="1.905" dy="1.905" layer="1" roundness="100"/>
<smd name="P$2" x="-2.54" y="1.27" dx="1.905" dy="1.905" layer="1" roundness="100"/>
<smd name="P$3" x="0" y="-1.27" dx="1.905" dy="1.905" layer="1" roundness="100"/>
<smd name="P$4" x="0" y="1.27" dx="1.905" dy="1.905" layer="1" roundness="100"/>
<smd name="P$5" x="2.54" y="-1.27" dx="1.905" dy="1.905" layer="1" roundness="100"/>
<smd name="P$6" x="2.54" y="1.27" dx="1.905" dy="1.905" layer="1" roundness="100"/>
</package>
</packages>
<symbols>
<symbol name="MA03-2">
<wire x1="3.81" y1="-5.08" x2="-3.81" y2="-5.08" width="0.4064" layer="94"/>
<wire x1="-3.81" y1="5.08" x2="-3.81" y2="-5.08" width="0.4064" layer="94"/>
<wire x1="3.81" y1="-5.08" x2="3.81" y2="5.08" width="0.4064" layer="94"/>
<wire x1="-3.81" y1="5.08" x2="3.81" y2="5.08" width="0.4064" layer="94"/>
<wire x1="1.27" y1="2.54" x2="2.54" y2="2.54" width="0.6096" layer="94"/>
<wire x1="1.27" y1="0" x2="2.54" y2="0" width="0.6096" layer="94"/>
<wire x1="1.27" y1="-2.54" x2="2.54" y2="-2.54" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="2.54" x2="-1.27" y2="2.54" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="0" x2="-1.27" y2="0" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="-2.54" x2="-1.27" y2="-2.54" width="0.6096" layer="94"/>
<text x="-3.81" y="-7.62" size="1.778" layer="96">&gt;VALUE</text>
<text x="-3.81" y="5.842" size="1.778" layer="95">&gt;NAME</text>
<pin name="1" x="7.62" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="3" x="7.62" y="0" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="5" x="7.62" y="2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="2" x="-7.62" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1"/>
<pin name="4" x="-7.62" y="0" visible="pad" length="middle" direction="pas" swaplevel="1"/>
<pin name="6" x="-7.62" y="2.54" visible="pad" length="middle" direction="pas" swaplevel="1"/>
<text x="2.286" y="-4.064" size="1.27" layer="94">1</text>
</symbol>
<symbol name="MOUNT-HOLE">
<wire x1="0" y1="1.27" x2="1.27" y2="0" width="1.524" layer="94" curve="-90" cap="flat"/>
<wire x1="-1.27" y1="0" x2="0" y2="-1.27" width="1.524" layer="94" curve="90" cap="flat"/>
<wire x1="-0.508" y1="0" x2="0.508" y2="0" width="0.0508" layer="94"/>
<wire x1="0" y1="0.508" x2="0" y2="-0.508" width="0.0508" layer="94"/>
<circle x="0" y="0" radius="2.032" width="0.0508" layer="94"/>
<circle x="0" y="0" radius="0.508" width="0.0508" layer="94"/>
<text x="2.032" y="0.5842" size="1.778" layer="95">&gt;NAME</text>
<text x="2.032" y="-2.4638" size="1.778" layer="96">&gt;VALUE</text>
</symbol>
<symbol name="M02">
<wire x1="3.81" y1="-2.54" x2="-2.54" y2="-2.54" width="0.4064" layer="94"/>
<wire x1="1.27" y1="2.54" x2="2.54" y2="2.54" width="0.6096" layer="94"/>
<wire x1="1.27" y1="0" x2="2.54" y2="0" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="5.08" x2="-2.54" y2="-2.54" width="0.4064" layer="94"/>
<wire x1="3.81" y1="-2.54" x2="3.81" y2="5.08" width="0.4064" layer="94"/>
<wire x1="-2.54" y1="5.08" x2="3.81" y2="5.08" width="0.4064" layer="94"/>
<text x="-2.54" y="-5.08" size="1.778" layer="96">&gt;VALUE</text>
<text x="-2.54" y="5.842" size="1.778" layer="95">&gt;NAME</text>
<pin name="1" x="7.62" y="0" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="2" x="7.62" y="2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
</symbol>
<symbol name="M03">
<wire x1="3.81" y1="-5.08" x2="-2.54" y2="-5.08" width="0.4064" layer="94"/>
<wire x1="1.27" y1="2.54" x2="2.54" y2="2.54" width="0.6096" layer="94"/>
<wire x1="1.27" y1="0" x2="2.54" y2="0" width="0.6096" layer="94"/>
<wire x1="1.27" y1="-2.54" x2="2.54" y2="-2.54" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="5.08" x2="-2.54" y2="-5.08" width="0.4064" layer="94"/>
<wire x1="3.81" y1="-5.08" x2="3.81" y2="5.08" width="0.4064" layer="94"/>
<wire x1="-2.54" y1="5.08" x2="3.81" y2="5.08" width="0.4064" layer="94"/>
<text x="-2.54" y="-7.62" size="1.778" layer="96">&gt;VALUE</text>
<text x="-2.54" y="5.842" size="1.778" layer="95">&gt;NAME</text>
<pin name="1" x="7.62" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="2" x="7.62" y="0" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="3" x="7.62" y="2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
</symbol>
<symbol name="OPAMP+-">
<wire x1="-5.08" y1="5.08" x2="-5.08" y2="-5.08" width="0.4064" layer="94"/>
<wire x1="-5.08" y1="-5.08" x2="5.08" y2="0" width="0.4064" layer="94"/>
<wire x1="5.08" y1="0" x2="-5.08" y2="5.08" width="0.4064" layer="94"/>
<wire x1="-3.81" y1="3.175" x2="-3.81" y2="1.905" width="0.1524" layer="94"/>
<wire x1="-4.445" y1="2.54" x2="-3.175" y2="2.54" width="0.1524" layer="94"/>
<wire x1="-4.445" y1="-2.54" x2="-3.175" y2="-2.54" width="0.1524" layer="94"/>
<text x="2.54" y="3.175" size="1.778" layer="95">&gt;NAME</text>
<text x="2.54" y="-5.08" size="1.778" layer="96">&gt;VALUE</text>
<text x="1.27" y="3.175" size="0.8128" layer="93" rot="R90">V+</text>
<text x="1.27" y="-4.445" size="0.8128" layer="93" rot="R90">V-</text>
<pin name="-IN" x="-7.62" y="-2.54" visible="pad" length="short" direction="in"/>
<pin name="+IN" x="-7.62" y="2.54" visible="pad" length="short" direction="in"/>
<pin name="OUT" x="7.62" y="0" visible="pad" length="short" direction="out" rot="R180"/>
<pin name="V+" x="0" y="7.62" visible="pad" length="middle" direction="pwr" rot="R270"/>
<pin name="V-" x="0" y="-7.62" visible="pad" length="middle" direction="pwr" rot="R90"/>
</symbol>
<symbol name="MLX75306">
<wire x1="-15.24" y1="19.05" x2="-15.24" y2="-21.59" width="0.254" layer="94"/>
<wire x1="-15.24" y1="-21.59" x2="25.4" y2="-21.59" width="0.254" layer="94"/>
<wire x1="25.4" y1="-21.59" x2="25.4" y2="19.05" width="0.254" layer="94"/>
<wire x1="25.4" y1="19.05" x2="6.35" y2="19.05" width="0.254" layer="94"/>
<wire x1="-15.24" y1="19.05" x2="3.81" y2="19.05" width="0.254" layer="94"/>
<wire x1="3.81" y1="19.05" x2="6.35" y2="19.05" width="0.254" layer="94" curve="180"/>
<text x="-12.065" y="19.685" size="1.778" layer="95">&gt;NAME</text>
<text x="-1.905" y="-24.13" size="1.778" layer="96">&gt;VALUE</text>
<pin name="SCLK" x="-17.78" y="-2.54" length="short"/>
<pin name="MISO" x="-17.78" y="-5.08" length="short"/>
<pin name="MOSI" x="-17.78" y="-7.62" length="short"/>
<pin name="VSS" x="-17.78" y="-10.16" length="short" direction="pwr"/>
<pin name="V3V3A" x="-17.78" y="-12.7" length="short" direction="pwr"/>
<pin name="V3V3B" x="-17.78" y="-15.24" length="short" direction="pwr"/>
<pin name="FRAME" x="-17.78" y="-17.78" length="short"/>
<pin name="CS" x="-17.78" y="-20.32" length="short"/>
<pin name="NA8" x="27.94" y="0" length="short" rot="R180"/>
<pin name="NA7" x="27.94" y="2.54" length="short" rot="R180"/>
<pin name="NA6" x="27.94" y="5.08" length="short" rot="R180"/>
<pin name="NA5" x="27.94" y="7.62" length="short" rot="R180"/>
<pin name="NA4" x="27.94" y="10.16" length="short" rot="R180"/>
<pin name="NA3" x="27.94" y="12.7" length="short" rot="R180"/>
<pin name="NA2" x="27.94" y="15.24" length="short" rot="R180"/>
<pin name="NA1" x="27.94" y="17.78" length="short" rot="R180"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="MA03-2" prefix="SV" uservalue="yes">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<gates>
<gate name="1" symbol="MA03-2" x="0" y="0"/>
</gates>
<devices>
<device name="P" package="MA03-2">
<connects>
<connect gate="1" pin="1" pad="1"/>
<connect gate="1" pin="2" pad="2"/>
<connect gate="1" pin="3" pad="3"/>
<connect gate="1" pin="4" pad="4"/>
<connect gate="1" pin="5" pad="5"/>
<connect gate="1" pin="6" pad="6"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="CONTACT" package="MA03-2CONTACT">
<connects>
<connect gate="1" pin="1" pad="1"/>
<connect gate="1" pin="2" pad="2"/>
<connect gate="1" pin="3" pad="3"/>
<connect gate="1" pin="4" pad="4"/>
<connect gate="1" pin="5" pad="5"/>
<connect gate="1" pin="6" pad="6"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="&quot;" package="MA03-2">
<connects>
<connect gate="1" pin="1" pad="1"/>
<connect gate="1" pin="2" pad="2"/>
<connect gate="1" pin="3" pad="3"/>
<connect gate="1" pin="4" pad="4"/>
<connect gate="1" pin="5" pad="5"/>
<connect gate="1" pin="6" pad="6"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="SMD" package="MA03-2SMT">
<connects>
<connect gate="1" pin="1" pad="P$1"/>
<connect gate="1" pin="2" pad="P$2"/>
<connect gate="1" pin="3" pad="P$3"/>
<connect gate="1" pin="4" pad="P$4"/>
<connect gate="1" pin="5" pad="P$5"/>
<connect gate="1" pin="6" pad="P$6"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="MOUNT-HOLE" prefix="H">
<description>&lt;b&gt;MOUNTING HOLE&lt;/b&gt; with drill center marker</description>
<gates>
<gate name="G$1" symbol="MOUNT-HOLE" x="0" y="0"/>
</gates>
<devices>
<device name="2.8" package="2,8">
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="3.0" package="3,0">
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="3.3" package="3,3">
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="3.6" package="3,6">
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="4.1" package="4,1">
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="4.5" package="4,5">
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="5.0" package="5,0">
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="3.2" package="3,2">
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="4.3" package="4,3">
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="5.5" package="5,5">
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="1.5" package="1.5">
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="2.0" package="2.0">
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="" package="SCREWHOLE2-56">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="M02" prefix="JP">
<description>Standard 2-pin 0.1" header. Use with &lt;br&gt;
- straight break away headers ( PRT-00116)&lt;br&gt;
- right angle break away headers (PRT-00553)&lt;br&gt;
- swiss pins (PRT-00743)&lt;br&gt;
- machine pins (PRT-00117)&lt;br&gt;
- female headers (PRT-00115)&lt;br&gt;&lt;br&gt;

 Molex polarized connector foot print use with: PRT-08233 with associated crimp pins and housings.&lt;br&gt;&lt;br&gt;

2.54_SCREWTERM for use with  PRT-10571.&lt;br&gt;&lt;br&gt;

3.5mm Screw Terminal footprints for  PRT-08084&lt;br&gt;&lt;br&gt;

5mm Screw Terminal footprints for use with PRT-08433</description>
<gates>
<gate name="G$1" symbol="M02" x="-2.54" y="0"/>
</gates>
<devices>
<device name="PTH" package="1X02">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="POLAR" package="MOLEX-1X2">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="3.5MM" package="SCREWTERMINAL-3.5MM-2">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="-JST-2MM-SMT" package="JST-2-SMD">
<connects>
<connect gate="G$1" pin="1" pad="2"/>
<connect gate="G$1" pin="2" pad="1"/>
</connects>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="CONN-08352"/>
</technology>
</technologies>
</device>
<device name="PTH2" package="1X02_BIG">
<connects>
<connect gate="G$1" pin="1" pad="P$1"/>
<connect gate="G$1" pin="2" pad="P$2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="4UCON-15767" package="JST-2-SMD-VERT">
<connects>
<connect gate="G$1" pin="1" pad="GND"/>
<connect gate="G$1" pin="2" pad="VCC"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="ROCKER" package="R_SW_TH">
<connects>
<connect gate="G$1" pin="1" pad="P$3"/>
<connect gate="G$1" pin="2" pad="P$4"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="5MM" package="SCREWTERMINAL-5MM-2">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="LOCK" package="1X02_LOCK">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="POLAR_LOCK" package="MOLEX-1X2_LOCK">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="LOCK_LONGPADS" package="1X02_LOCK_LONGPADS">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="3.5MM_LOCK" package="SCREWTERMINAL-3.5MM-2_LOCK">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="PTH3" package="1X02_LONGPADS">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="1X02_NO_SILK" package="1X02_NO_SILK">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="JST-PTH-2" package="JST-2-PTH">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name="">
<attribute name="SKU" value="PRT-09914" constant="no"/>
</technology>
</technologies>
</device>
<device name="PTH4" package="1X02_XTRA_BIG">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="POGO_PIN_HOLES_ONLY" package="1X02_PP_HOLES_ONLY">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="3.5MM-NO_SILK" package="SCREWTERMINAL-3.5MM-2-NS">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="-JST-2-PTH-NO_SILK" package="JST-2-PTH-NS">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="JST-PTH-2-KIT" package="JST-2-PTH-KIT">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="SPRING-2.54-RA" package="SPRINGTERMINAL-2.54MM-2">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="2.54MM_SCREWTERM" package="1X02_2.54_SCREWTERM">
<connects>
<connect gate="G$1" pin="1" pad="P1"/>
<connect gate="G$1" pin="2" pad="P2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="JST-PTH-VERT" package="JST-2-PTH-VERT">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="M03" prefix="JP" uservalue="yes">
<description>&lt;b&gt;Header 3&lt;/b&gt;
Standard 3-pin 0.1" header. Use with straight break away headers (SKU : PRT-00116), right angle break away headers (PRT-00553), swiss pins (PRT-00743), machine pins (PRT-00117), and female headers (PRT-00115). Molex polarized connector foot print use with SKU : PRT-08232 with associated crimp pins and housings.</description>
<gates>
<gate name="G$1" symbol="M03" x="-2.54" y="0"/>
</gates>
<devices>
<device name="PTH" package="1X03">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="POLAR" package="MOLEX-1X3">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="SCREW" package="SCREWTERMINAL-3.5MM-3">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="LOCK" package="1X03_LOCK">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="LOCK_LONGPADS" package="1X03_LOCK_LONGPADS">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="POLAR_LOCK" package="MOLEX-1X3_LOCK">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="SCREW_LOCK" package="SCREWTERMINAL-3.5MM-3_LOCK.007S">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="1X03_NO_SILK" package="1X03_NO_SILK">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="LONGPADS" package="1X03_LONGPADS">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="JST-PTH" package="JST-3-PTH">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
</connects>
<technologies>
<technology name="">
<attribute name="SKU" value="PRT-09915" constant="no"/>
</technology>
</technologies>
</device>
<device name="POGO_PIN_HOLES_ONLY" package="1X03_PP_HOLES_ONLY">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="-SCREW-5MM" package="SCREWTERMINAL-5MM-3">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="LOCK_NO_SILK" package="1X03_LOCK_NO_SILK">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="JST-SMD" package="JST-3-SMD">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="SMD" package="1X03-1MM-RA">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="SMD_RA_FEMALE" package="1X03_SMD_RA_FEMALE">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
</connects>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="CONN-10926"/>
<attribute name="VALUE" value="1x3 RA Female .1&quot;"/>
</technology>
</technologies>
</device>
<device name="SMD_RA_MALE" package="1X03_SMD_RA_MALE">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
</connects>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="CONN-10925"/>
</technology>
</technologies>
</device>
<device name="SMD_RA_MALE_POST" package="1X03_SMD_RA_MALE_POST">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="JST-PTH-VERT" package="JST-3-PTH-VERT">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="1X03_SMD_RA_MALE_POST_SMALLER" package="1X03_SMD_RA_MALE_POST_SMALLER">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="1X03_SMD_RA_MALE_POST_SMALLEST" package="1X03_SMD_RA_MALE_POST_SMALLEST">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="SCREW2.54" package="1X03_2.54_SCREWTERM">
<connects>
<connect gate="G$1" pin="1" pad="P1"/>
<connect gate="G$1" pin="2" pad="P2"/>
<connect gate="G$1" pin="3" pad="P3"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="MCP6001" prefix="IC">
<description>The Microchip Technology Inc. MCP6001/2/4 family of
operational   amplifiers   (op   amps)   is   specifically
designed for general-purpose applications. This family
has  a  1 MHz  Gain  Bandwidth  Product  (GBWP)  and
90°  phase  margin  (typ.).  It  also  maintains  45°  phase
margin (typ.) with a 500 pF capacitive load. This family
operates from a single supply voltage as low as 1.8V,
while    drawing    100 μA    (typ.)    quiescent    current.
Additionally,   the   MCP6001/2/4   supports   rail-to-rail
input  and  output  swing,  with  a  common  mode  input
voltage range of V
DD
+300mV to V
SS
– 300 mV. This
family   of   op   amps   is   designed   with   Microchip’s
advanced CMOS process.</description>
<gates>
<gate name="G$1" symbol="OPAMP+-" x="2.54" y="0"/>
</gates>
<devices>
<device name="OT" package="SOT23-5">
<connects>
<connect gate="G$1" pin="+IN" pad="3"/>
<connect gate="G$1" pin="-IN" pad="4"/>
<connect gate="G$1" pin="OUT" pad="1"/>
<connect gate="G$1" pin="V+" pad="5"/>
<connect gate="G$1" pin="V-" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="LT" package="MCP6001_SC70-5">
<connects>
<connect gate="G$1" pin="+IN" pad="3"/>
<connect gate="G$1" pin="-IN" pad="4"/>
<connect gate="G$1" pin="OUT" pad="1"/>
<connect gate="G$1" pin="V+" pad="5"/>
<connect gate="G$1" pin="V-" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="MLX75306" prefix="IC">
<gates>
<gate name="G$1" symbol="MLX75306" x="-2.54" y="0"/>
</gates>
<devices>
<device name="" package="SO16NBC">
<connects>
<connect gate="G$1" pin="CS" pad="8"/>
<connect gate="G$1" pin="FRAME" pad="7"/>
<connect gate="G$1" pin="MISO" pad="2"/>
<connect gate="G$1" pin="MOSI" pad="3"/>
<connect gate="G$1" pin="NA1" pad="9"/>
<connect gate="G$1" pin="NA2" pad="10"/>
<connect gate="G$1" pin="NA3" pad="11"/>
<connect gate="G$1" pin="NA4" pad="12"/>
<connect gate="G$1" pin="NA5" pad="13"/>
<connect gate="G$1" pin="NA6" pad="14"/>
<connect gate="G$1" pin="NA7" pad="15"/>
<connect gate="G$1" pin="NA8" pad="16"/>
<connect gate="G$1" pin="SCLK" pad="1"/>
<connect gate="G$1" pin="V3V3A" pad="5"/>
<connect gate="G$1" pin="V3V3B" pad="6"/>
<connect gate="G$1" pin="VSS" pad="4"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="supply1">
<packages>
</packages>
<symbols>
<symbol name="GND">
<wire x1="-1.905" y1="0" x2="1.905" y2="0" width="0.254" layer="94"/>
<text x="-2.54" y="-2.54" size="1.778" layer="96">&gt;VALUE</text>
<pin name="GND" x="0" y="2.54" visible="off" length="short" direction="sup" rot="R270"/>
</symbol>
<symbol name="+5V">
<wire x1="1.27" y1="-1.905" x2="0" y2="0" width="0.254" layer="94"/>
<wire x1="0" y1="0" x2="-1.27" y2="-1.905" width="0.254" layer="94"/>
<text x="-2.54" y="-5.08" size="1.778" layer="96" rot="R90">&gt;VALUE</text>
<pin name="+5V" x="0" y="-2.54" visible="off" length="short" direction="sup" rot="R90"/>
</symbol>
<symbol name="VCC">
<wire x1="1.27" y1="-1.905" x2="0" y2="0" width="0.254" layer="94"/>
<wire x1="0" y1="0" x2="-1.27" y2="-1.905" width="0.254" layer="94"/>
<text x="-2.54" y="-2.54" size="1.778" layer="96" rot="R90">&gt;VALUE</text>
<pin name="VCC" x="0" y="-2.54" visible="off" length="short" direction="sup" rot="R90"/>
</symbol>
<symbol name="+3V3">
<wire x1="1.27" y1="-1.905" x2="0" y2="0" width="0.254" layer="94"/>
<wire x1="0" y1="0" x2="-1.27" y2="-1.905" width="0.254" layer="94"/>
<text x="-2.54" y="-5.08" size="1.778" layer="96" rot="R90">&gt;VALUE</text>
<pin name="+3V3" x="0" y="-2.54" visible="off" length="short" direction="sup" rot="R90"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="GND" prefix="GND">
<description>&lt;b&gt;SUPPLY SYMBOL&lt;/b&gt;</description>
<gates>
<gate name="1" symbol="GND" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="+5V" prefix="P+">
<description>&lt;b&gt;SUPPLY SYMBOL&lt;/b&gt;</description>
<gates>
<gate name="1" symbol="+5V" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="VCC" prefix="P+">
<description>&lt;b&gt;SUPPLY SYMBOL&lt;/b&gt;</description>
<gates>
<gate name="VCC" symbol="VCC" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="+3V3" prefix="+3V3">
<description>&lt;b&gt;SUPPLY SYMBOL&lt;/b&gt;</description>
<gates>
<gate name="G$1" symbol="+3V3" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="rcl">
<packages>
<package name="M1206">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
MELF 0.25 W</description>
<wire x1="-2.473" y1="1.483" x2="2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-1.483" x2="-2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-1.483" x2="-2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="1.483" x2="2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="1.143" y1="0.8382" x2="-1.143" y2="0.8382" width="0.1524" layer="51"/>
<wire x1="1.143" y1="-0.8382" x2="-1.143" y2="-0.8382" width="0.1524" layer="51"/>
<smd name="1" x="-1.4" y="0" dx="1.6" dy="2" layer="1"/>
<smd name="2" x="1.4" y="0" dx="1.6" dy="2" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.7018" y1="-0.9144" x2="-1.1176" y2="0.9144" layer="51"/>
<rectangle x1="1.1176" y1="-0.9144" x2="1.7018" y2="0.9144" layer="51"/>
<rectangle x1="-0.3" y1="-0.8001" x2="0.3" y2="0.8001" layer="35"/>
</package>
<package name="M3216">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
MELF 0.25 W</description>
<wire x1="-2.473" y1="1.483" x2="2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-1.483" x2="-2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-1.483" x2="-2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="1.483" x2="2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="1.143" y1="0.8382" x2="-1.143" y2="0.8382" width="0.1524" layer="51"/>
<wire x1="1.143" y1="-0.8382" x2="-1.143" y2="-0.8382" width="0.1524" layer="51"/>
<smd name="1" x="-1.4" y="0" dx="1.6" dy="2" layer="1"/>
<smd name="2" x="1.4" y="0" dx="1.6" dy="2" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.7018" y1="-0.9144" x2="-1.1176" y2="0.9144" layer="51"/>
<rectangle x1="1.1176" y1="-0.9144" x2="1.7018" y2="0.9144" layer="51"/>
<rectangle x1="-0.3" y1="-0.8001" x2="0.3" y2="0.8001" layer="35"/>
</package>
<package name="V234/12">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type V234, grid 12.5 mm</description>
<wire x1="-4.953" y1="1.524" x2="-4.699" y2="1.778" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.699" y1="1.778" x2="4.953" y2="1.524" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.699" y1="-1.778" x2="4.953" y2="-1.524" width="0.1524" layer="21" curve="90"/>
<wire x1="-4.953" y1="-1.524" x2="-4.699" y2="-1.778" width="0.1524" layer="21" curve="90"/>
<wire x1="-4.699" y1="1.778" x2="4.699" y2="1.778" width="0.1524" layer="21"/>
<wire x1="-4.953" y1="1.524" x2="-4.953" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="4.699" y1="-1.778" x2="-4.699" y2="-1.778" width="0.1524" layer="21"/>
<wire x1="4.953" y1="1.524" x2="4.953" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="6.35" y1="0" x2="5.461" y2="0" width="0.8128" layer="51"/>
<wire x1="-6.35" y1="0" x2="-5.461" y2="0" width="0.8128" layer="51"/>
<pad name="1" x="-6.35" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="6.35" y="0" drill="1.016" shape="octagon"/>
<text x="-4.953" y="2.159" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.81" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="4.953" y1="-0.4064" x2="5.4102" y2="0.4064" layer="21"/>
<rectangle x1="-5.4102" y1="-0.4064" x2="-4.953" y2="0.4064" layer="21"/>
</package>
<package name="V235/17">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type V235, grid 17.78 mm</description>
<wire x1="-6.731" y1="2.921" x2="6.731" y2="2.921" width="0.1524" layer="21"/>
<wire x1="-7.112" y1="2.54" x2="-7.112" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="6.731" y1="-2.921" x2="-6.731" y2="-2.921" width="0.1524" layer="21"/>
<wire x1="7.112" y1="2.54" x2="7.112" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="8.89" y1="0" x2="7.874" y2="0" width="1.016" layer="51"/>
<wire x1="-7.874" y1="0" x2="-8.89" y2="0" width="1.016" layer="51"/>
<wire x1="-7.112" y1="-2.54" x2="-6.731" y2="-2.921" width="0.1524" layer="21" curve="90"/>
<wire x1="6.731" y1="2.921" x2="7.112" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<wire x1="6.731" y1="-2.921" x2="7.112" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-7.112" y1="2.54" x2="-6.731" y2="2.921" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-8.89" y="0" drill="1.1938" shape="octagon"/>
<pad name="2" x="8.89" y="0" drill="1.1938" shape="octagon"/>
<text x="-6.858" y="3.302" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.842" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="7.112" y1="-0.508" x2="7.747" y2="0.508" layer="21"/>
<rectangle x1="-7.747" y1="-0.508" x2="-7.112" y2="0.508" layer="21"/>
</package>
<package name="V526-0">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type V526-0, grid 2.5 mm</description>
<wire x1="-2.54" y1="1.016" x2="-2.286" y2="1.27" width="0.1524" layer="21" curve="-90"/>
<wire x1="2.286" y1="1.27" x2="2.54" y2="1.016" width="0.1524" layer="21" curve="-90"/>
<wire x1="2.286" y1="-1.27" x2="2.54" y2="-1.016" width="0.1524" layer="21" curve="90"/>
<wire x1="-2.54" y1="-1.016" x2="-2.286" y2="-1.27" width="0.1524" layer="21" curve="90"/>
<wire x1="2.286" y1="1.27" x2="-2.286" y2="1.27" width="0.1524" layer="21"/>
<wire x1="2.54" y1="-1.016" x2="2.54" y2="1.016" width="0.1524" layer="21"/>
<wire x1="-2.286" y1="-1.27" x2="2.286" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="1.016" x2="-2.54" y2="-1.016" width="0.1524" layer="21"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.413" y="1.651" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.413" y="-2.794" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="MINI_MELF-0102AX">
<description>&lt;b&gt;Mini MELF 0102 Axial&lt;/b&gt;</description>
<circle x="0" y="0" radius="0.6" width="0" layer="51"/>
<circle x="0" y="0" radius="0.6" width="0" layer="52"/>
<smd name="1" x="0" y="0" dx="1.9" dy="1.9" layer="1" roundness="100"/>
<smd name="2" x="0" y="0" dx="1.9" dy="1.9" layer="16" roundness="100"/>
<text x="-1.27" y="0.9525" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.2225" size="1.27" layer="27">&gt;VALUE</text>
<hole x="0" y="0" drill="1.3"/>
</package>
<package name="MINI_MELF-0102R">
<description>&lt;b&gt;CECC Size RC2211&lt;/b&gt; Reflow Soldering&lt;p&gt;
source Beyschlag</description>
<wire x1="-1" y1="-0.5" x2="1" y2="-0.5" width="0.2032" layer="51"/>
<wire x1="1" y1="-0.5" x2="1" y2="0.5" width="0.2032" layer="51"/>
<wire x1="1" y1="0.5" x2="-1" y2="0.5" width="0.2032" layer="51"/>
<wire x1="-1" y1="0.5" x2="-1" y2="-0.5" width="0.2032" layer="51"/>
<smd name="1" x="-0.9" y="0" dx="0.5" dy="1.3" layer="1"/>
<smd name="2" x="0.9" y="0" dx="0.5" dy="1.3" layer="1"/>
<text x="-1.27" y="0.9525" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.2225" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="MINI_MELF-0102W">
<description>&lt;b&gt;CECC Size RC2211&lt;/b&gt; Wave Soldering&lt;p&gt;
source Beyschlag</description>
<wire x1="-1" y1="-0.5" x2="1" y2="-0.5" width="0.2032" layer="51"/>
<wire x1="1" y1="-0.5" x2="1" y2="0.5" width="0.2032" layer="51"/>
<wire x1="1" y1="0.5" x2="-1" y2="0.5" width="0.2032" layer="51"/>
<wire x1="-1" y1="0.5" x2="-1" y2="-0.5" width="0.2032" layer="51"/>
<smd name="1" x="-0.95" y="0" dx="0.6" dy="1.3" layer="1"/>
<smd name="2" x="0.95" y="0" dx="0.6" dy="1.3" layer="1"/>
<text x="-1.27" y="0.9525" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.2225" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="RDH/15">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type RDH, grid 15 mm</description>
<wire x1="-7.62" y1="0" x2="-6.858" y2="0" width="0.8128" layer="51"/>
<wire x1="-6.096" y1="3.048" x2="-5.207" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-4.953" y1="2.794" x2="-5.207" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-6.096" y1="-3.048" x2="-5.207" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-4.953" y1="-2.794" x2="-5.207" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="4.953" y1="2.794" x2="5.207" y2="3.048" width="0.1524" layer="21"/>
<wire x1="4.953" y1="2.794" x2="-4.953" y2="2.794" width="0.1524" layer="21"/>
<wire x1="4.953" y1="-2.794" x2="5.207" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="4.953" y1="-2.794" x2="-4.953" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="6.096" y1="3.048" x2="5.207" y2="3.048" width="0.1524" layer="21"/>
<wire x1="6.096" y1="-3.048" x2="5.207" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-6.477" y1="-2.667" x2="-6.477" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-6.477" y1="1.016" x2="-6.477" y2="-1.016" width="0.1524" layer="51"/>
<wire x1="-6.477" y1="1.016" x2="-6.477" y2="2.667" width="0.1524" layer="21"/>
<wire x1="6.477" y1="-2.667" x2="6.477" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="6.477" y1="1.016" x2="6.477" y2="-1.016" width="0.1524" layer="51"/>
<wire x1="6.477" y1="1.016" x2="6.477" y2="2.667" width="0.1524" layer="21"/>
<wire x1="6.858" y1="0" x2="7.62" y2="0" width="0.8128" layer="51"/>
<wire x1="-6.477" y1="2.667" x2="-6.096" y2="3.048" width="0.1524" layer="21" curve="-90"/>
<wire x1="6.096" y1="3.048" x2="6.477" y2="2.667" width="0.1524" layer="21" curve="-90"/>
<wire x1="-6.477" y1="-2.667" x2="-6.096" y2="-3.048" width="0.1524" layer="21" curve="90"/>
<wire x1="6.096" y1="-3.048" x2="6.477" y2="-2.667" width="0.1524" layer="21" curve="90"/>
<pad name="1" x="-7.62" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="7.62" y="0" drill="1.016" shape="octagon"/>
<text x="-6.35" y="3.4544" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-4.318" y="-0.5842" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="4.572" y="-1.7272" size="1.27" layer="51" ratio="10" rot="R90">RDH</text>
<rectangle x1="-6.7564" y1="-0.4064" x2="-6.4516" y2="0.4064" layer="51"/>
<rectangle x1="6.4516" y1="-0.4064" x2="6.7564" y2="0.4064" layer="51"/>
</package>
<package name="R0201">
<description>&lt;b&gt;RESISTOR&lt;/b&gt; chip&lt;p&gt;
Source: http://www.vishay.com/docs/20008/dcrcw.pdf</description>
<smd name="1" x="-0.255" y="0" dx="0.28" dy="0.43" layer="1"/>
<smd name="2" x="0.255" y="0" dx="0.28" dy="0.43" layer="1"/>
<text x="-0.635" y="0.635" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-1.905" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.3" y1="-0.15" x2="-0.15" y2="0.15" layer="51"/>
<rectangle x1="0.15" y1="-0.15" x2="0.3" y2="0.15" layer="51"/>
<rectangle x1="-0.15" y1="-0.15" x2="0.15" y2="0.15" layer="21"/>
</package>
<package name="R0402">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-0.245" y1="0.224" x2="0.245" y2="0.224" width="0.1524" layer="51"/>
<wire x1="0.245" y1="-0.224" x2="-0.245" y2="-0.224" width="0.1524" layer="51"/>
<wire x1="-1.473" y1="0.483" x2="1.473" y2="0.483" width="0.0508" layer="39"/>
<wire x1="1.473" y1="0.483" x2="1.473" y2="-0.483" width="0.0508" layer="39"/>
<wire x1="1.473" y1="-0.483" x2="-1.473" y2="-0.483" width="0.0508" layer="39"/>
<wire x1="-1.473" y1="-0.483" x2="-1.473" y2="0.483" width="0.0508" layer="39"/>
<smd name="1" x="-0.65" y="0" dx="0.7" dy="0.9" layer="1"/>
<smd name="2" x="0.65" y="0" dx="0.7" dy="0.9" layer="1"/>
<text x="-0.635" y="0.635" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-1.905" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.554" y1="-0.3048" x2="-0.254" y2="0.2951" layer="51"/>
<rectangle x1="0.2588" y1="-0.3048" x2="0.5588" y2="0.2951" layer="51"/>
<rectangle x1="-0.1999" y1="-0.4001" x2="0.1999" y2="0.4001" layer="35"/>
</package>
<package name="R0603">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-0.432" y1="-0.356" x2="0.432" y2="-0.356" width="0.1524" layer="51"/>
<wire x1="0.432" y1="0.356" x2="-0.432" y2="0.356" width="0.1524" layer="51"/>
<wire x1="-1.473" y1="0.983" x2="1.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.473" y1="0.983" x2="1.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.473" y1="-0.983" x2="-1.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.473" y1="-0.983" x2="-1.473" y2="0.983" width="0.0508" layer="39"/>
<smd name="1" x="-0.85" y="0" dx="1" dy="1.1" layer="1"/>
<smd name="2" x="0.85" y="0" dx="1" dy="1.1" layer="1"/>
<text x="-0.635" y="0.635" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-1.905" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="0.4318" y1="-0.4318" x2="0.8382" y2="0.4318" layer="51"/>
<rectangle x1="-0.8382" y1="-0.4318" x2="-0.4318" y2="0.4318" layer="51"/>
<rectangle x1="-0.1999" y1="-0.4001" x2="0.1999" y2="0.4001" layer="35"/>
</package>
<package name="R0805">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;</description>
<wire x1="-0.41" y1="0.635" x2="0.41" y2="0.635" width="0.1524" layer="51"/>
<wire x1="-0.41" y1="-0.635" x2="0.41" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="-1.973" y1="0.983" x2="1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="0.983" x2="1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="-0.983" x2="-1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.973" y1="-0.983" x2="-1.973" y2="0.983" width="0.0508" layer="39"/>
<smd name="1" x="-0.95" y="0" dx="1.3" dy="1.5" layer="1"/>
<smd name="2" x="0.95" y="0" dx="1.3" dy="1.5" layer="1"/>
<text x="-0.635" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="0.4064" y1="-0.6985" x2="1.0564" y2="0.7015" layer="51"/>
<rectangle x1="-1.0668" y1="-0.6985" x2="-0.4168" y2="0.7015" layer="51"/>
<rectangle x1="-0.1999" y1="-0.5001" x2="0.1999" y2="0.5001" layer="35"/>
</package>
<package name="R1206">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="0.9525" y1="-0.8128" x2="-0.9652" y2="-0.8128" width="0.1524" layer="51"/>
<wire x1="0.9525" y1="0.8128" x2="-0.9652" y2="0.8128" width="0.1524" layer="51"/>
<wire x1="-2.473" y1="0.983" x2="2.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="0.983" x2="2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-0.983" x2="-2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-0.983" x2="-2.473" y2="0.983" width="0.0508" layer="39"/>
<smd name="2" x="1.422" y="0" dx="1.6" dy="1.803" layer="1"/>
<smd name="1" x="-1.422" y="0" dx="1.6" dy="1.803" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.6891" y1="-0.8763" x2="-0.9525" y2="0.8763" layer="51"/>
<rectangle x1="0.9525" y1="-0.8763" x2="1.6891" y2="0.8763" layer="51"/>
<rectangle x1="-0.3" y1="-0.7" x2="0.3" y2="0.7" layer="35"/>
</package>
<package name="R1206W">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
wave soldering</description>
<wire x1="-0.913" y1="0.8" x2="0.888" y2="0.8" width="0.1524" layer="51"/>
<wire x1="-0.913" y1="-0.8" x2="0.888" y2="-0.8" width="0.1524" layer="51"/>
<wire x1="-2.473" y1="0.983" x2="2.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="0.983" x2="2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-0.983" x2="-2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-0.983" x2="-2.473" y2="0.983" width="0.0508" layer="39"/>
<smd name="1" x="-1.499" y="0" dx="1.8" dy="1.2" layer="1"/>
<smd name="2" x="1.499" y="0" dx="1.8" dy="1.2" layer="1"/>
<text x="-1.905" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.651" y1="-0.8763" x2="-0.9009" y2="0.8738" layer="51"/>
<rectangle x1="0.889" y1="-0.8763" x2="1.6391" y2="0.8738" layer="51"/>
<rectangle x1="-0.3" y1="-0.7" x2="0.3" y2="0.7" layer="35"/>
</package>
<package name="R1210">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-0.913" y1="1.219" x2="0.939" y2="1.219" width="0.1524" layer="51"/>
<wire x1="-0.913" y1="-1.219" x2="0.939" y2="-1.219" width="0.1524" layer="51"/>
<wire x1="-2.473" y1="1.483" x2="2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="1.483" x2="2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-1.483" x2="-2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-1.483" x2="-2.473" y2="1.483" width="0.0508" layer="39"/>
<smd name="1" x="-1.4" y="0" dx="1.6" dy="2.7" layer="1"/>
<smd name="2" x="1.4" y="0" dx="1.6" dy="2.7" layer="1"/>
<text x="-2.54" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.651" y1="-1.3081" x2="-0.9009" y2="1.2918" layer="51"/>
<rectangle x1="0.9144" y1="-1.3081" x2="1.6645" y2="1.2918" layer="51"/>
<rectangle x1="-0.3" y1="-0.8999" x2="0.3" y2="0.8999" layer="35"/>
</package>
<package name="R1210W">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
wave soldering</description>
<wire x1="-0.913" y1="1.219" x2="0.939" y2="1.219" width="0.1524" layer="51"/>
<wire x1="-0.913" y1="-1.219" x2="0.939" y2="-1.219" width="0.1524" layer="51"/>
<wire x1="-2.473" y1="1.483" x2="2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="1.483" x2="2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-1.483" x2="-2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-1.483" x2="-2.473" y2="1.483" width="0.0508" layer="39"/>
<smd name="1" x="-1.499" y="0" dx="1.8" dy="1.8" layer="1"/>
<smd name="2" x="1.499" y="0" dx="1.8" dy="1.8" layer="1"/>
<text x="-2.54" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.651" y1="-1.3081" x2="-0.9009" y2="1.2918" layer="51"/>
<rectangle x1="0.9144" y1="-1.3081" x2="1.6645" y2="1.2918" layer="51"/>
<rectangle x1="-0.3" y1="-0.8001" x2="0.3" y2="0.8001" layer="35"/>
</package>
<package name="R2010">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-1.662" y1="1.245" x2="1.662" y2="1.245" width="0.1524" layer="51"/>
<wire x1="-1.637" y1="-1.245" x2="1.687" y2="-1.245" width="0.1524" layer="51"/>
<wire x1="-3.473" y1="1.483" x2="3.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="3.473" y1="1.483" x2="3.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="3.473" y1="-1.483" x2="-3.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-3.473" y1="-1.483" x2="-3.473" y2="1.483" width="0.0508" layer="39"/>
<smd name="1" x="-2.2" y="0" dx="1.8" dy="2.7" layer="1"/>
<smd name="2" x="2.2" y="0" dx="1.8" dy="2.7" layer="1"/>
<text x="-3.175" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-3.175" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-2.4892" y1="-1.3208" x2="-1.6393" y2="1.3292" layer="51"/>
<rectangle x1="1.651" y1="-1.3208" x2="2.5009" y2="1.3292" layer="51"/>
</package>
<package name="R2010W">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
wave soldering</description>
<wire x1="-1.662" y1="1.245" x2="1.662" y2="1.245" width="0.1524" layer="51"/>
<wire x1="-1.637" y1="-1.245" x2="1.687" y2="-1.245" width="0.1524" layer="51"/>
<wire x1="-3.473" y1="1.483" x2="3.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="3.473" y1="1.483" x2="3.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="3.473" y1="-1.483" x2="-3.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-3.473" y1="-1.483" x2="-3.473" y2="1.483" width="0.0508" layer="39"/>
<smd name="1" x="-2.311" y="0" dx="2" dy="1.8" layer="1"/>
<smd name="2" x="2.311" y="0" dx="2" dy="1.8" layer="1"/>
<text x="-2.54" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-2.4892" y1="-1.3208" x2="-1.6393" y2="1.3292" layer="51"/>
<rectangle x1="1.651" y1="-1.3208" x2="2.5009" y2="1.3292" layer="51"/>
</package>
<package name="R2012">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-0.41" y1="0.635" x2="0.41" y2="0.635" width="0.1524" layer="51"/>
<wire x1="-0.41" y1="-0.635" x2="0.41" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="-1.973" y1="0.983" x2="1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="0.983" x2="1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="-0.983" x2="-1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.973" y1="-0.983" x2="-1.973" y2="0.983" width="0.0508" layer="39"/>
<smd name="1" x="-0.85" y="0" dx="1.3" dy="1.5" layer="1"/>
<smd name="2" x="0.85" y="0" dx="1.3" dy="1.5" layer="1"/>
<text x="-0.635" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="0.4064" y1="-0.6985" x2="1.0564" y2="0.7015" layer="51"/>
<rectangle x1="-1.0668" y1="-0.6985" x2="-0.4168" y2="0.7015" layer="51"/>
<rectangle x1="-0.1001" y1="-0.5999" x2="0.1001" y2="0.5999" layer="35"/>
</package>
<package name="R2012W">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
wave soldering</description>
<wire x1="-0.41" y1="0.635" x2="0.41" y2="0.635" width="0.1524" layer="51"/>
<wire x1="-0.41" y1="-0.635" x2="0.41" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="-1.973" y1="0.983" x2="1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="0.983" x2="1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="-0.983" x2="-1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.973" y1="-0.983" x2="-1.973" y2="0.983" width="0.0508" layer="39"/>
<smd name="1" x="-0.94" y="0" dx="1.5" dy="1" layer="1"/>
<smd name="2" x="0.94" y="0" dx="1.5" dy="1" layer="1"/>
<text x="-0.635" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="0.4064" y1="-0.6985" x2="1.0564" y2="0.7015" layer="51"/>
<rectangle x1="-1.0668" y1="-0.6985" x2="-0.4168" y2="0.7015" layer="51"/>
<rectangle x1="-0.1001" y1="-0.5999" x2="0.1001" y2="0.5999" layer="35"/>
</package>
<package name="R2512">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-2.362" y1="1.473" x2="2.387" y2="1.473" width="0.1524" layer="51"/>
<wire x1="-2.362" y1="-1.473" x2="2.387" y2="-1.473" width="0.1524" layer="51"/>
<wire x1="-3.973" y1="1.983" x2="3.973" y2="1.983" width="0.0508" layer="39"/>
<wire x1="3.973" y1="1.983" x2="3.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="3.973" y1="-1.983" x2="-3.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="-3.973" y1="-1.983" x2="-3.973" y2="1.983" width="0.0508" layer="39"/>
<smd name="1" x="-2.8" y="0" dx="1.8" dy="3.2" layer="1"/>
<smd name="2" x="2.8" y="0" dx="1.8" dy="3.2" layer="1"/>
<text x="-2.54" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-3.2004" y1="-1.5494" x2="-2.3505" y2="1.5507" layer="51"/>
<rectangle x1="2.3622" y1="-1.5494" x2="3.2121" y2="1.5507" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
</package>
<package name="R2512W">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
wave soldering</description>
<wire x1="-2.362" y1="1.473" x2="2.387" y2="1.473" width="0.1524" layer="51"/>
<wire x1="-2.362" y1="-1.473" x2="2.387" y2="-1.473" width="0.1524" layer="51"/>
<wire x1="-3.973" y1="1.983" x2="3.973" y2="1.983" width="0.0508" layer="39"/>
<wire x1="3.973" y1="1.983" x2="3.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="3.973" y1="-1.983" x2="-3.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="-3.973" y1="-1.983" x2="-3.973" y2="1.983" width="0.0508" layer="39"/>
<smd name="1" x="-2.896" y="0" dx="2" dy="2.1" layer="1"/>
<smd name="2" x="2.896" y="0" dx="2" dy="2.1" layer="1"/>
<text x="-1.905" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-3.2004" y1="-1.5494" x2="-2.3505" y2="1.5507" layer="51"/>
<rectangle x1="2.3622" y1="-1.5494" x2="3.2121" y2="1.5507" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
</package>
<package name="R3216">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-0.913" y1="0.8" x2="0.888" y2="0.8" width="0.1524" layer="51"/>
<wire x1="-0.913" y1="-0.8" x2="0.888" y2="-0.8" width="0.1524" layer="51"/>
<wire x1="-2.473" y1="0.983" x2="2.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="0.983" x2="2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-0.983" x2="-2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-0.983" x2="-2.473" y2="0.983" width="0.0508" layer="39"/>
<smd name="1" x="-1.4" y="0" dx="1.6" dy="1.8" layer="1"/>
<smd name="2" x="1.4" y="0" dx="1.6" dy="1.8" layer="1"/>
<text x="-1.905" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.651" y1="-0.8763" x2="-0.9009" y2="0.8738" layer="51"/>
<rectangle x1="0.889" y1="-0.8763" x2="1.6391" y2="0.8738" layer="51"/>
<rectangle x1="-0.3" y1="-0.7" x2="0.3" y2="0.7" layer="35"/>
</package>
<package name="R3216W">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
wave soldering</description>
<wire x1="-0.913" y1="0.8" x2="0.888" y2="0.8" width="0.1524" layer="51"/>
<wire x1="-0.913" y1="-0.8" x2="0.888" y2="-0.8" width="0.1524" layer="51"/>
<wire x1="-2.473" y1="0.983" x2="2.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="0.983" x2="2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-0.983" x2="-2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-0.983" x2="-2.473" y2="0.983" width="0.0508" layer="39"/>
<smd name="1" x="-1.499" y="0" dx="1.8" dy="1.2" layer="1"/>
<smd name="2" x="1.499" y="0" dx="1.8" dy="1.2" layer="1"/>
<text x="-1.905" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.651" y1="-0.8763" x2="-0.9009" y2="0.8738" layer="51"/>
<rectangle x1="0.889" y1="-0.8763" x2="1.6391" y2="0.8738" layer="51"/>
<rectangle x1="-0.3" y1="-0.7" x2="0.3" y2="0.7" layer="35"/>
</package>
<package name="R3225">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-0.913" y1="1.219" x2="0.939" y2="1.219" width="0.1524" layer="51"/>
<wire x1="-0.913" y1="-1.219" x2="0.939" y2="-1.219" width="0.1524" layer="51"/>
<wire x1="-2.473" y1="1.483" x2="2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="1.483" x2="2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-1.483" x2="-2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-1.483" x2="-2.473" y2="1.483" width="0.0508" layer="39"/>
<smd name="1" x="-1.4" y="0" dx="1.6" dy="2.7" layer="1"/>
<smd name="2" x="1.4" y="0" dx="1.6" dy="2.7" layer="1"/>
<text x="-2.54" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.651" y1="-1.3081" x2="-0.9009" y2="1.2918" layer="51"/>
<rectangle x1="0.9144" y1="-1.3081" x2="1.6645" y2="1.2918" layer="51"/>
<rectangle x1="-0.3" y1="-1" x2="0.3" y2="1" layer="35"/>
</package>
<package name="R3225W">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
wave soldering</description>
<wire x1="-0.913" y1="1.219" x2="0.939" y2="1.219" width="0.1524" layer="51"/>
<wire x1="-0.913" y1="-1.219" x2="0.939" y2="-1.219" width="0.1524" layer="51"/>
<wire x1="-2.473" y1="1.483" x2="2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="1.483" x2="2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-1.483" x2="-2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-1.483" x2="-2.473" y2="1.483" width="0.0508" layer="39"/>
<smd name="1" x="-1.499" y="0" dx="1.8" dy="1.8" layer="1"/>
<smd name="2" x="1.499" y="0" dx="1.8" dy="1.8" layer="1"/>
<text x="-1.905" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.651" y1="-1.3081" x2="-0.9009" y2="1.2918" layer="51"/>
<rectangle x1="0.9144" y1="-1.3081" x2="1.6645" y2="1.2918" layer="51"/>
<rectangle x1="-0.3" y1="-1" x2="0.3" y2="1" layer="35"/>
</package>
<package name="R5025">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-1.662" y1="1.245" x2="1.662" y2="1.245" width="0.1524" layer="51"/>
<wire x1="-1.637" y1="-1.245" x2="1.687" y2="-1.245" width="0.1524" layer="51"/>
<wire x1="-3.473" y1="1.483" x2="3.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="3.473" y1="1.483" x2="3.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="3.473" y1="-1.483" x2="-3.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-3.473" y1="-1.483" x2="-3.473" y2="1.483" width="0.0508" layer="39"/>
<smd name="1" x="-2.2" y="0" dx="1.8" dy="2.7" layer="1"/>
<smd name="2" x="2.2" y="0" dx="1.8" dy="2.7" layer="1"/>
<text x="-3.175" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-3.175" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-2.4892" y1="-1.3208" x2="-1.6393" y2="1.3292" layer="51"/>
<rectangle x1="1.651" y1="-1.3208" x2="2.5009" y2="1.3292" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
</package>
<package name="R5025W">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
wave soldering</description>
<wire x1="-1.662" y1="1.245" x2="1.662" y2="1.245" width="0.1524" layer="51"/>
<wire x1="-1.637" y1="-1.245" x2="1.687" y2="-1.245" width="0.1524" layer="51"/>
<wire x1="-3.473" y1="1.483" x2="3.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="3.473" y1="1.483" x2="3.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="3.473" y1="-1.483" x2="-3.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-3.473" y1="-1.483" x2="-3.473" y2="1.483" width="0.0508" layer="39"/>
<smd name="1" x="-2.311" y="0" dx="2" dy="1.8" layer="1"/>
<smd name="2" x="2.311" y="0" dx="2" dy="1.8" layer="1"/>
<text x="-3.175" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-3.175" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-2.4892" y1="-1.3208" x2="-1.6393" y2="1.3292" layer="51"/>
<rectangle x1="1.651" y1="-1.3208" x2="2.5009" y2="1.3292" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
</package>
<package name="R6332">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
Source: http://download.siliconexpert.com/pdfs/2005/02/24/Semi_Ap/2/VSH/Resistor/dcrcwfre.pdf</description>
<wire x1="-2.362" y1="1.473" x2="2.387" y2="1.473" width="0.1524" layer="51"/>
<wire x1="-2.362" y1="-1.473" x2="2.387" y2="-1.473" width="0.1524" layer="51"/>
<wire x1="-3.973" y1="1.983" x2="3.973" y2="1.983" width="0.0508" layer="39"/>
<wire x1="3.973" y1="1.983" x2="3.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="3.973" y1="-1.983" x2="-3.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="-3.973" y1="-1.983" x2="-3.973" y2="1.983" width="0.0508" layer="39"/>
<smd name="1" x="-3.1" y="0" dx="1" dy="3.2" layer="1"/>
<smd name="2" x="3.1" y="0" dx="1" dy="3.2" layer="1"/>
<text x="-2.54" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-3.2004" y1="-1.5494" x2="-2.3505" y2="1.5507" layer="51"/>
<rectangle x1="2.3622" y1="-1.5494" x2="3.2121" y2="1.5507" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
</package>
<package name="R6332W">
<description>&lt;b&gt;RESISTOR&lt;/b&gt; wave soldering&lt;p&gt;
Source: http://download.siliconexpert.com/pdfs/2005/02/24/Semi_Ap/2/VSH/Resistor/dcrcwfre.pdf</description>
<wire x1="-2.362" y1="1.473" x2="2.387" y2="1.473" width="0.1524" layer="51"/>
<wire x1="-2.362" y1="-1.473" x2="2.387" y2="-1.473" width="0.1524" layer="51"/>
<wire x1="-3.973" y1="1.983" x2="3.973" y2="1.983" width="0.0508" layer="39"/>
<wire x1="3.973" y1="1.983" x2="3.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="3.973" y1="-1.983" x2="-3.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="-3.973" y1="-1.983" x2="-3.973" y2="1.983" width="0.0508" layer="39"/>
<smd name="1" x="-3.196" y="0" dx="1.2" dy="3.2" layer="1"/>
<smd name="2" x="3.196" y="0" dx="1.2" dy="3.2" layer="1"/>
<text x="-2.54" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-3.2004" y1="-1.5494" x2="-2.3505" y2="1.5507" layer="51"/>
<rectangle x1="2.3622" y1="-1.5494" x2="3.2121" y2="1.5507" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
</package>
<package name="M0805">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
MELF 0.10 W</description>
<wire x1="-1.973" y1="0.983" x2="1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="-0.983" x2="-1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.973" y1="-0.983" x2="-1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="0.983" x2="1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="0.7112" y1="0.635" x2="-0.7112" y2="0.635" width="0.1524" layer="51"/>
<wire x1="0.7112" y1="-0.635" x2="-0.7112" y2="-0.635" width="0.1524" layer="51"/>
<smd name="1" x="-0.95" y="0" dx="1.3" dy="1.6" layer="1"/>
<smd name="2" x="0.95" y="0" dx="1.3" dy="1.6" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.0414" y1="-0.7112" x2="-0.6858" y2="0.7112" layer="51"/>
<rectangle x1="0.6858" y1="-0.7112" x2="1.0414" y2="0.7112" layer="51"/>
<rectangle x1="-0.1999" y1="-0.5999" x2="0.1999" y2="0.5999" layer="35"/>
</package>
<package name="M1406">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
MELF 0.12 W</description>
<wire x1="-2.973" y1="0.983" x2="2.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.973" y1="-0.983" x2="-2.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-2.973" y1="-0.983" x2="-2.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.973" y1="0.983" x2="2.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.3208" y1="0.762" x2="-1.3208" y2="0.762" width="0.1524" layer="51"/>
<wire x1="1.3208" y1="-0.762" x2="-1.3208" y2="-0.762" width="0.1524" layer="51"/>
<smd name="1" x="-1.7" y="0" dx="1.4" dy="1.8" layer="1"/>
<smd name="2" x="1.7" y="0" dx="1.4" dy="1.8" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.8542" y1="-0.8382" x2="-1.2954" y2="0.8382" layer="51"/>
<rectangle x1="1.2954" y1="-0.8382" x2="1.8542" y2="0.8382" layer="51"/>
<rectangle x1="-0.3" y1="-0.7" x2="0.3" y2="0.7" layer="35"/>
</package>
<package name="M2012">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
MELF 0.10 W</description>
<wire x1="-1.973" y1="0.983" x2="1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="-0.983" x2="-1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.973" y1="-0.983" x2="-1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="0.983" x2="1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="0.7112" y1="0.635" x2="-0.7112" y2="0.635" width="0.1524" layer="51"/>
<wire x1="0.7112" y1="-0.635" x2="-0.7112" y2="-0.635" width="0.1524" layer="51"/>
<smd name="1" x="-0.95" y="0" dx="1.3" dy="1.6" layer="1"/>
<smd name="2" x="0.95" y="0" dx="1.3" dy="1.6" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.0414" y1="-0.7112" x2="-0.6858" y2="0.7112" layer="51"/>
<rectangle x1="0.6858" y1="-0.7112" x2="1.0414" y2="0.7112" layer="51"/>
<rectangle x1="-0.1999" y1="-0.5999" x2="0.1999" y2="0.5999" layer="35"/>
</package>
<package name="M2309">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
MELF 0.25 W</description>
<wire x1="-4.473" y1="1.483" x2="4.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="4.473" y1="-1.483" x2="-4.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-4.473" y1="-1.483" x2="-4.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="4.473" y1="1.483" x2="4.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="2.413" y1="1.1684" x2="-2.4384" y2="1.1684" width="0.1524" layer="51"/>
<wire x1="2.413" y1="-1.1684" x2="-2.413" y2="-1.1684" width="0.1524" layer="51"/>
<smd name="1" x="-2.85" y="0" dx="1.5" dy="2.6" layer="1"/>
<smd name="2" x="2.85" y="0" dx="1.5" dy="2.6" layer="1"/>
<text x="-1.905" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-3.048" y1="-1.2446" x2="-2.3876" y2="1.2446" layer="51"/>
<rectangle x1="2.3876" y1="-1.2446" x2="3.048" y2="1.2446" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
</package>
<package name="M3516">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
MELF 0.12 W</description>
<wire x1="-2.973" y1="0.983" x2="2.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.973" y1="-0.983" x2="-2.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-2.973" y1="-0.983" x2="-2.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.973" y1="0.983" x2="2.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.3208" y1="0.762" x2="-1.3208" y2="0.762" width="0.1524" layer="51"/>
<wire x1="1.3208" y1="-0.762" x2="-1.3208" y2="-0.762" width="0.1524" layer="51"/>
<smd name="1" x="-1.7" y="0" dx="1.4" dy="1.8" layer="1"/>
<smd name="2" x="1.7" y="0" dx="1.4" dy="1.8" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.8542" y1="-0.8382" x2="-1.2954" y2="0.8382" layer="51"/>
<rectangle x1="1.2954" y1="-0.8382" x2="1.8542" y2="0.8382" layer="51"/>
<rectangle x1="-0.4001" y1="-0.7" x2="0.4001" y2="0.7" layer="35"/>
</package>
<package name="M5923">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
MELF 0.25 W</description>
<wire x1="-4.473" y1="1.483" x2="4.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="4.473" y1="-1.483" x2="-4.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-4.473" y1="-1.483" x2="-4.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="4.473" y1="1.483" x2="4.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="2.413" y1="1.1684" x2="-2.4384" y2="1.1684" width="0.1524" layer="51"/>
<wire x1="2.413" y1="-1.1684" x2="-2.413" y2="-1.1684" width="0.1524" layer="51"/>
<smd name="1" x="-2.85" y="0" dx="1.5" dy="2.6" layer="1"/>
<smd name="2" x="2.85" y="0" dx="1.5" dy="2.6" layer="1"/>
<text x="-1.905" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-3.048" y1="-1.2446" x2="-2.3876" y2="1.2446" layer="51"/>
<rectangle x1="2.3876" y1="-1.2446" x2="3.048" y2="1.2446" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
</package>
<package name="0207/10">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0207, grid 10 mm</description>
<wire x1="5.08" y1="0" x2="4.064" y2="0" width="0.6096" layer="51"/>
<wire x1="-5.08" y1="0" x2="-4.064" y2="0" width="0.6096" layer="51"/>
<wire x1="-3.175" y1="0.889" x2="-2.921" y2="1.143" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.175" y1="-0.889" x2="-2.921" y2="-1.143" width="0.1524" layer="21" curve="90"/>
<wire x1="2.921" y1="-1.143" x2="3.175" y2="-0.889" width="0.1524" layer="21" curve="90"/>
<wire x1="2.921" y1="1.143" x2="3.175" y2="0.889" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.175" y1="-0.889" x2="-3.175" y2="0.889" width="0.1524" layer="21"/>
<wire x1="-2.921" y1="1.143" x2="-2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="1.016" x2="-2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-2.921" y1="-1.143" x2="-2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="-1.016" x2="-2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="-2.413" y2="1.016" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="-2.413" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="2.921" y1="1.143" x2="2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.921" y1="-1.143" x2="2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-0.889" x2="3.175" y2="0.889" width="0.1524" layer="21"/>
<pad name="1" x="-5.08" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="5.08" y="0" drill="0.8128" shape="octagon"/>
<text x="-3.048" y="1.524" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.2606" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="3.175" y1="-0.3048" x2="4.0386" y2="0.3048" layer="21"/>
<rectangle x1="-4.0386" y1="-0.3048" x2="-3.175" y2="0.3048" layer="21"/>
</package>
<package name="0207/12">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0207, grid 12 mm</description>
<wire x1="6.35" y1="0" x2="5.334" y2="0" width="0.6096" layer="51"/>
<wire x1="-6.35" y1="0" x2="-5.334" y2="0" width="0.6096" layer="51"/>
<wire x1="-3.175" y1="0.889" x2="-2.921" y2="1.143" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.175" y1="-0.889" x2="-2.921" y2="-1.143" width="0.1524" layer="21" curve="90"/>
<wire x1="2.921" y1="-1.143" x2="3.175" y2="-0.889" width="0.1524" layer="21" curve="90"/>
<wire x1="2.921" y1="1.143" x2="3.175" y2="0.889" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.175" y1="-0.889" x2="-3.175" y2="0.889" width="0.1524" layer="21"/>
<wire x1="-2.921" y1="1.143" x2="-2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="1.016" x2="-2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-2.921" y1="-1.143" x2="-2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="-1.016" x2="-2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="-2.413" y2="1.016" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="-2.413" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="2.921" y1="1.143" x2="2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.921" y1="-1.143" x2="2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-0.889" x2="3.175" y2="0.889" width="0.1524" layer="21"/>
<wire x1="4.445" y1="0" x2="4.064" y2="0" width="0.6096" layer="21"/>
<wire x1="-4.445" y1="0" x2="-4.064" y2="0" width="0.6096" layer="21"/>
<pad name="1" x="-6.35" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="6.35" y="0" drill="0.8128" shape="octagon"/>
<text x="-3.175" y="1.397" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.286" y="-0.6858" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="3.175" y1="-0.3048" x2="4.0386" y2="0.3048" layer="21"/>
<rectangle x1="-4.0386" y1="-0.3048" x2="-3.175" y2="0.3048" layer="21"/>
<rectangle x1="4.445" y1="-0.3048" x2="5.3086" y2="0.3048" layer="21"/>
<rectangle x1="-5.3086" y1="-0.3048" x2="-4.445" y2="0.3048" layer="21"/>
</package>
<package name="0207/15">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0207, grid 15mm</description>
<wire x1="7.62" y1="0" x2="6.604" y2="0" width="0.6096" layer="51"/>
<wire x1="-7.62" y1="0" x2="-6.604" y2="0" width="0.6096" layer="51"/>
<wire x1="-3.175" y1="0.889" x2="-2.921" y2="1.143" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.175" y1="-0.889" x2="-2.921" y2="-1.143" width="0.1524" layer="21" curve="90"/>
<wire x1="2.921" y1="-1.143" x2="3.175" y2="-0.889" width="0.1524" layer="21" curve="90"/>
<wire x1="2.921" y1="1.143" x2="3.175" y2="0.889" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.175" y1="-0.889" x2="-3.175" y2="0.889" width="0.1524" layer="21"/>
<wire x1="-2.921" y1="1.143" x2="-2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="1.016" x2="-2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-2.921" y1="-1.143" x2="-2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="-1.016" x2="-2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="-2.413" y2="1.016" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="-2.413" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="2.921" y1="1.143" x2="2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.921" y1="-1.143" x2="2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-0.889" x2="3.175" y2="0.889" width="0.1524" layer="21"/>
<wire x1="5.715" y1="0" x2="4.064" y2="0" width="0.6096" layer="21"/>
<wire x1="-5.715" y1="0" x2="-4.064" y2="0" width="0.6096" layer="21"/>
<pad name="1" x="-7.62" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="7.62" y="0" drill="0.8128" shape="octagon"/>
<text x="-3.175" y="1.397" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.286" y="-0.6858" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="3.175" y1="-0.3048" x2="4.0386" y2="0.3048" layer="21"/>
<rectangle x1="-4.0386" y1="-0.3048" x2="-3.175" y2="0.3048" layer="21"/>
<rectangle x1="5.715" y1="-0.3048" x2="6.5786" y2="0.3048" layer="21"/>
<rectangle x1="-6.5786" y1="-0.3048" x2="-5.715" y2="0.3048" layer="21"/>
</package>
<package name="0207/2V">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0207, grid 2.5 mm</description>
<wire x1="-1.27" y1="0" x2="-0.381" y2="0" width="0.6096" layer="51"/>
<wire x1="-0.254" y1="0" x2="0.254" y2="0" width="0.6096" layer="21"/>
<wire x1="0.381" y1="0" x2="1.27" y2="0" width="0.6096" layer="51"/>
<circle x="-1.27" y="0" radius="1.27" width="0.1524" layer="21"/>
<circle x="-1.27" y="0" radius="1.016" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-0.0508" y="1.016" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-0.0508" y="-2.2352" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="0207/5V">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0207, grid 5 mm</description>
<wire x1="-2.54" y1="0" x2="-0.889" y2="0" width="0.6096" layer="51"/>
<wire x1="-0.762" y1="0" x2="0.762" y2="0" width="0.6096" layer="21"/>
<wire x1="0.889" y1="0" x2="2.54" y2="0" width="0.6096" layer="51"/>
<circle x="-2.54" y="0" radius="1.27" width="0.1016" layer="21"/>
<circle x="-2.54" y="0" radius="1.016" width="0.1524" layer="51"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" shape="octagon"/>
<text x="-1.143" y="0.889" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.143" y="-2.159" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="0207/7">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0207, grid 7.5 mm</description>
<wire x1="-3.81" y1="0" x2="-3.429" y2="0" width="0.6096" layer="51"/>
<wire x1="-3.175" y1="0.889" x2="-2.921" y2="1.143" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.175" y1="-0.889" x2="-2.921" y2="-1.143" width="0.1524" layer="21" curve="90"/>
<wire x1="2.921" y1="-1.143" x2="3.175" y2="-0.889" width="0.1524" layer="21" curve="90"/>
<wire x1="2.921" y1="1.143" x2="3.175" y2="0.889" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.175" y1="-0.889" x2="-3.175" y2="0.889" width="0.1524" layer="51"/>
<wire x1="-2.921" y1="1.143" x2="-2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="1.016" x2="-2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-2.921" y1="-1.143" x2="-2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="-1.016" x2="-2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="-2.413" y2="1.016" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="-2.413" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="2.921" y1="1.143" x2="2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.921" y1="-1.143" x2="2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-0.889" x2="3.175" y2="0.889" width="0.1524" layer="51"/>
<wire x1="3.429" y1="0" x2="3.81" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-3.81" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="3.81" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.54" y="1.397" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.286" y="-0.5588" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-3.429" y1="-0.3048" x2="-3.175" y2="0.3048" layer="51"/>
<rectangle x1="3.175" y1="-0.3048" x2="3.429" y2="0.3048" layer="51"/>
</package>
<package name="0309/10">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0309, grid 10mm</description>
<wire x1="-4.699" y1="0" x2="-5.08" y2="0" width="0.6096" layer="51"/>
<wire x1="-4.318" y1="1.27" x2="-4.064" y2="1.524" width="0.1524" layer="21" curve="-90"/>
<wire x1="-4.318" y1="-1.27" x2="-4.064" y2="-1.524" width="0.1524" layer="21" curve="90"/>
<wire x1="4.064" y1="-1.524" x2="4.318" y2="-1.27" width="0.1524" layer="21" curve="90"/>
<wire x1="4.064" y1="1.524" x2="4.318" y2="1.27" width="0.1524" layer="21" curve="-90"/>
<wire x1="-4.318" y1="-1.27" x2="-4.318" y2="1.27" width="0.1524" layer="51"/>
<wire x1="-4.064" y1="1.524" x2="-3.429" y2="1.524" width="0.1524" layer="21"/>
<wire x1="-3.302" y1="1.397" x2="-3.429" y2="1.524" width="0.1524" layer="21"/>
<wire x1="-4.064" y1="-1.524" x2="-3.429" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="-3.302" y1="-1.397" x2="-3.429" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="3.302" y1="1.397" x2="3.429" y2="1.524" width="0.1524" layer="21"/>
<wire x1="3.302" y1="1.397" x2="-3.302" y2="1.397" width="0.1524" layer="21"/>
<wire x1="3.302" y1="-1.397" x2="3.429" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="3.302" y1="-1.397" x2="-3.302" y2="-1.397" width="0.1524" layer="21"/>
<wire x1="4.064" y1="1.524" x2="3.429" y2="1.524" width="0.1524" layer="21"/>
<wire x1="4.064" y1="-1.524" x2="3.429" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="4.318" y1="-1.27" x2="4.318" y2="1.27" width="0.1524" layer="51"/>
<wire x1="5.08" y1="0" x2="4.699" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-5.08" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="5.08" y="0" drill="0.8128" shape="octagon"/>
<text x="-4.191" y="1.905" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="-0.6858" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-4.6228" y1="-0.3048" x2="-4.318" y2="0.3048" layer="51"/>
<rectangle x1="4.318" y1="-0.3048" x2="4.6228" y2="0.3048" layer="51"/>
</package>
<package name="0309/12">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0309, grid 12.5 mm</description>
<wire x1="6.35" y1="0" x2="5.08" y2="0" width="0.6096" layer="51"/>
<wire x1="-6.35" y1="0" x2="-5.08" y2="0" width="0.6096" layer="51"/>
<wire x1="-4.318" y1="1.27" x2="-4.064" y2="1.524" width="0.1524" layer="21" curve="-90"/>
<wire x1="-4.318" y1="-1.27" x2="-4.064" y2="-1.524" width="0.1524" layer="21" curve="90"/>
<wire x1="4.064" y1="-1.524" x2="4.318" y2="-1.27" width="0.1524" layer="21" curve="90"/>
<wire x1="4.064" y1="1.524" x2="4.318" y2="1.27" width="0.1524" layer="21" curve="-90"/>
<wire x1="-4.318" y1="-1.27" x2="-4.318" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-4.064" y1="1.524" x2="-3.429" y2="1.524" width="0.1524" layer="21"/>
<wire x1="-3.302" y1="1.397" x2="-3.429" y2="1.524" width="0.1524" layer="21"/>
<wire x1="-4.064" y1="-1.524" x2="-3.429" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="-3.302" y1="-1.397" x2="-3.429" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="3.302" y1="1.397" x2="3.429" y2="1.524" width="0.1524" layer="21"/>
<wire x1="3.302" y1="1.397" x2="-3.302" y2="1.397" width="0.1524" layer="21"/>
<wire x1="3.302" y1="-1.397" x2="3.429" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="3.302" y1="-1.397" x2="-3.302" y2="-1.397" width="0.1524" layer="21"/>
<wire x1="4.064" y1="1.524" x2="3.429" y2="1.524" width="0.1524" layer="21"/>
<wire x1="4.064" y1="-1.524" x2="3.429" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="4.318" y1="-1.27" x2="4.318" y2="1.27" width="0.1524" layer="21"/>
<pad name="1" x="-6.35" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="6.35" y="0" drill="0.8128" shape="octagon"/>
<text x="-4.191" y="1.905" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="-0.6858" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="4.318" y1="-0.3048" x2="5.1816" y2="0.3048" layer="21"/>
<rectangle x1="-5.1816" y1="-0.3048" x2="-4.318" y2="0.3048" layer="21"/>
</package>
<package name="0411/12">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0411, grid 12.5 mm</description>
<wire x1="6.35" y1="0" x2="5.461" y2="0" width="0.762" layer="51"/>
<wire x1="-6.35" y1="0" x2="-5.461" y2="0" width="0.762" layer="51"/>
<wire x1="5.08" y1="-1.651" x2="5.08" y2="1.651" width="0.1524" layer="21"/>
<wire x1="4.699" y1="2.032" x2="5.08" y2="1.651" width="0.1524" layer="21" curve="-90"/>
<wire x1="-5.08" y1="-1.651" x2="-4.699" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="4.699" y1="-2.032" x2="5.08" y2="-1.651" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.08" y1="1.651" x2="-4.699" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.699" y1="2.032" x2="4.064" y2="2.032" width="0.1524" layer="21"/>
<wire x1="3.937" y1="1.905" x2="4.064" y2="2.032" width="0.1524" layer="21"/>
<wire x1="4.699" y1="-2.032" x2="4.064" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="3.937" y1="-1.905" x2="4.064" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="-3.937" y1="1.905" x2="-4.064" y2="2.032" width="0.1524" layer="21"/>
<wire x1="-3.937" y1="1.905" x2="3.937" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-3.937" y1="-1.905" x2="-4.064" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="-3.937" y1="-1.905" x2="3.937" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="1.651" x2="-5.08" y2="-1.651" width="0.1524" layer="21"/>
<wire x1="-4.699" y1="2.032" x2="-4.064" y2="2.032" width="0.1524" layer="21"/>
<wire x1="-4.699" y1="-2.032" x2="-4.064" y2="-2.032" width="0.1524" layer="21"/>
<pad name="1" x="-6.35" y="0" drill="0.9144" shape="octagon"/>
<pad name="2" x="6.35" y="0" drill="0.9144" shape="octagon"/>
<text x="-5.08" y="2.413" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.5814" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-5.3594" y1="-0.381" x2="-5.08" y2="0.381" layer="21"/>
<rectangle x1="5.08" y1="-0.381" x2="5.3594" y2="0.381" layer="21"/>
</package>
<package name="0411/15">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0411, grid 15 mm</description>
<wire x1="5.08" y1="-1.651" x2="5.08" y2="1.651" width="0.1524" layer="21"/>
<wire x1="4.699" y1="2.032" x2="5.08" y2="1.651" width="0.1524" layer="21" curve="-90"/>
<wire x1="-5.08" y1="-1.651" x2="-4.699" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="4.699" y1="-2.032" x2="5.08" y2="-1.651" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.08" y1="1.651" x2="-4.699" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.699" y1="2.032" x2="4.064" y2="2.032" width="0.1524" layer="21"/>
<wire x1="3.937" y1="1.905" x2="4.064" y2="2.032" width="0.1524" layer="21"/>
<wire x1="4.699" y1="-2.032" x2="4.064" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="3.937" y1="-1.905" x2="4.064" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="-3.937" y1="1.905" x2="-4.064" y2="2.032" width="0.1524" layer="21"/>
<wire x1="-3.937" y1="1.905" x2="3.937" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-3.937" y1="-1.905" x2="-4.064" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="-3.937" y1="-1.905" x2="3.937" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="1.651" x2="-5.08" y2="-1.651" width="0.1524" layer="21"/>
<wire x1="-4.699" y1="2.032" x2="-4.064" y2="2.032" width="0.1524" layer="21"/>
<wire x1="-4.699" y1="-2.032" x2="-4.064" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="-7.62" y1="0" x2="-6.35" y2="0" width="0.762" layer="51"/>
<wire x1="6.35" y1="0" x2="7.62" y2="0" width="0.762" layer="51"/>
<pad name="1" x="-7.62" y="0" drill="0.9144" shape="octagon"/>
<pad name="2" x="7.62" y="0" drill="0.9144" shape="octagon"/>
<text x="-5.08" y="2.413" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.5814" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="5.08" y1="-0.381" x2="6.477" y2="0.381" layer="21"/>
<rectangle x1="-6.477" y1="-0.381" x2="-5.08" y2="0.381" layer="21"/>
</package>
<package name="0411V">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0411, grid 3.81 mm</description>
<wire x1="1.27" y1="0" x2="0.3048" y2="0" width="0.762" layer="51"/>
<wire x1="-1.5748" y1="0" x2="-2.54" y2="0" width="0.762" layer="51"/>
<circle x="-2.54" y="0" radius="2.032" width="0.1524" layer="21"/>
<circle x="-2.54" y="0" radius="1.016" width="0.1524" layer="51"/>
<pad name="1" x="-2.54" y="0" drill="0.9144" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.9144" shape="octagon"/>
<text x="-0.508" y="1.143" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-0.5334" y="-2.413" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-1.4732" y1="-0.381" x2="0.2032" y2="0.381" layer="21"/>
</package>
<package name="0414/15">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0414, grid 15 mm</description>
<wire x1="7.62" y1="0" x2="6.604" y2="0" width="0.8128" layer="51"/>
<wire x1="-7.62" y1="0" x2="-6.604" y2="0" width="0.8128" layer="51"/>
<wire x1="-6.096" y1="1.905" x2="-5.842" y2="2.159" width="0.1524" layer="21" curve="-90"/>
<wire x1="-6.096" y1="-1.905" x2="-5.842" y2="-2.159" width="0.1524" layer="21" curve="90"/>
<wire x1="5.842" y1="-2.159" x2="6.096" y2="-1.905" width="0.1524" layer="21" curve="90"/>
<wire x1="5.842" y1="2.159" x2="6.096" y2="1.905" width="0.1524" layer="21" curve="-90"/>
<wire x1="-6.096" y1="-1.905" x2="-6.096" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-5.842" y1="2.159" x2="-4.953" y2="2.159" width="0.1524" layer="21"/>
<wire x1="-4.826" y1="2.032" x2="-4.953" y2="2.159" width="0.1524" layer="21"/>
<wire x1="-5.842" y1="-2.159" x2="-4.953" y2="-2.159" width="0.1524" layer="21"/>
<wire x1="-4.826" y1="-2.032" x2="-4.953" y2="-2.159" width="0.1524" layer="21"/>
<wire x1="4.826" y1="2.032" x2="4.953" y2="2.159" width="0.1524" layer="21"/>
<wire x1="4.826" y1="2.032" x2="-4.826" y2="2.032" width="0.1524" layer="21"/>
<wire x1="4.826" y1="-2.032" x2="4.953" y2="-2.159" width="0.1524" layer="21"/>
<wire x1="4.826" y1="-2.032" x2="-4.826" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="5.842" y1="2.159" x2="4.953" y2="2.159" width="0.1524" layer="21"/>
<wire x1="5.842" y1="-2.159" x2="4.953" y2="-2.159" width="0.1524" layer="21"/>
<wire x1="6.096" y1="-1.905" x2="6.096" y2="1.905" width="0.1524" layer="21"/>
<pad name="1" x="-7.62" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="7.62" y="0" drill="1.016" shape="octagon"/>
<text x="-6.096" y="2.5654" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-4.318" y="-0.5842" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="6.096" y1="-0.4064" x2="6.5024" y2="0.4064" layer="21"/>
<rectangle x1="-6.5024" y1="-0.4064" x2="-6.096" y2="0.4064" layer="21"/>
</package>
<package name="0414V">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0414, grid 5 mm</description>
<wire x1="2.54" y1="0" x2="1.397" y2="0" width="0.8128" layer="51"/>
<wire x1="-2.54" y1="0" x2="-1.397" y2="0" width="0.8128" layer="51"/>
<circle x="-2.54" y="0" radius="2.159" width="0.1524" layer="21"/>
<circle x="-2.54" y="0" radius="1.143" width="0.1524" layer="51"/>
<pad name="1" x="-2.54" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="1.016" shape="octagon"/>
<text x="-0.381" y="1.1684" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-0.381" y="-2.3622" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-1.2954" y1="-0.4064" x2="1.2954" y2="0.4064" layer="21"/>
</package>
<package name="0617/17">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0617, grid 17.5 mm</description>
<wire x1="-8.89" y1="0" x2="-8.636" y2="0" width="0.8128" layer="51"/>
<wire x1="-7.874" y1="3.048" x2="-6.985" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="2.794" x2="-6.985" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-7.874" y1="-3.048" x2="-6.985" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="-2.794" x2="-6.985" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="6.731" y1="2.794" x2="6.985" y2="3.048" width="0.1524" layer="21"/>
<wire x1="6.731" y1="2.794" x2="-6.731" y2="2.794" width="0.1524" layer="21"/>
<wire x1="6.731" y1="-2.794" x2="6.985" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="6.731" y1="-2.794" x2="-6.731" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="7.874" y1="3.048" x2="6.985" y2="3.048" width="0.1524" layer="21"/>
<wire x1="7.874" y1="-3.048" x2="6.985" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-8.255" y1="-2.667" x2="-8.255" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-8.255" y1="1.016" x2="-8.255" y2="-1.016" width="0.1524" layer="51"/>
<wire x1="-8.255" y1="1.016" x2="-8.255" y2="2.667" width="0.1524" layer="21"/>
<wire x1="8.255" y1="-2.667" x2="8.255" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="8.255" y1="1.016" x2="8.255" y2="-1.016" width="0.1524" layer="51"/>
<wire x1="8.255" y1="1.016" x2="8.255" y2="2.667" width="0.1524" layer="21"/>
<wire x1="8.636" y1="0" x2="8.89" y2="0" width="0.8128" layer="51"/>
<wire x1="-8.255" y1="2.667" x2="-7.874" y2="3.048" width="0.1524" layer="21" curve="-90"/>
<wire x1="7.874" y1="3.048" x2="8.255" y2="2.667" width="0.1524" layer="21" curve="-90"/>
<wire x1="-8.255" y1="-2.667" x2="-7.874" y2="-3.048" width="0.1524" layer="21" curve="90"/>
<wire x1="7.874" y1="-3.048" x2="8.255" y2="-2.667" width="0.1524" layer="21" curve="90"/>
<pad name="1" x="-8.89" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="8.89" y="0" drill="1.016" shape="octagon"/>
<text x="-8.128" y="3.4544" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-6.096" y="-0.7112" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-8.5344" y1="-0.4064" x2="-8.2296" y2="0.4064" layer="51"/>
<rectangle x1="8.2296" y1="-0.4064" x2="8.5344" y2="0.4064" layer="51"/>
</package>
<package name="0617/22">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0617, grid 22.5 mm</description>
<wire x1="-10.287" y1="0" x2="-11.43" y2="0" width="0.8128" layer="51"/>
<wire x1="-8.255" y1="-2.667" x2="-8.255" y2="2.667" width="0.1524" layer="21"/>
<wire x1="-7.874" y1="3.048" x2="-6.985" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="2.794" x2="-6.985" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-7.874" y1="-3.048" x2="-6.985" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="-2.794" x2="-6.985" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="6.731" y1="2.794" x2="6.985" y2="3.048" width="0.1524" layer="21"/>
<wire x1="6.731" y1="2.794" x2="-6.731" y2="2.794" width="0.1524" layer="21"/>
<wire x1="6.731" y1="-2.794" x2="6.985" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="6.731" y1="-2.794" x2="-6.731" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="7.874" y1="3.048" x2="6.985" y2="3.048" width="0.1524" layer="21"/>
<wire x1="7.874" y1="-3.048" x2="6.985" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="8.255" y1="-2.667" x2="8.255" y2="2.667" width="0.1524" layer="21"/>
<wire x1="11.43" y1="0" x2="10.287" y2="0" width="0.8128" layer="51"/>
<wire x1="-8.255" y1="2.667" x2="-7.874" y2="3.048" width="0.1524" layer="21" curve="-90"/>
<wire x1="-8.255" y1="-2.667" x2="-7.874" y2="-3.048" width="0.1524" layer="21" curve="90"/>
<wire x1="7.874" y1="3.048" x2="8.255" y2="2.667" width="0.1524" layer="21" curve="-90"/>
<wire x1="7.874" y1="-3.048" x2="8.255" y2="-2.667" width="0.1524" layer="21" curve="90"/>
<pad name="1" x="-11.43" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="11.43" y="0" drill="1.016" shape="octagon"/>
<text x="-8.255" y="3.4544" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-6.477" y="-0.5842" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-10.1854" y1="-0.4064" x2="-8.255" y2="0.4064" layer="21"/>
<rectangle x1="8.255" y1="-0.4064" x2="10.1854" y2="0.4064" layer="21"/>
</package>
<package name="0617V">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0617, grid 5 mm</description>
<wire x1="-2.54" y1="0" x2="-1.27" y2="0" width="0.8128" layer="51"/>
<wire x1="1.27" y1="0" x2="2.54" y2="0" width="0.8128" layer="51"/>
<circle x="-2.54" y="0" radius="3.048" width="0.1524" layer="21"/>
<circle x="-2.54" y="0" radius="1.143" width="0.1524" layer="51"/>
<pad name="1" x="-2.54" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="1.016" shape="octagon"/>
<text x="0.635" y="1.4224" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="0.635" y="-2.6162" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-1.3208" y1="-0.4064" x2="1.3208" y2="0.4064" layer="21"/>
</package>
<package name="0922/22">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0922, grid 22.5 mm</description>
<wire x1="11.43" y1="0" x2="10.795" y2="0" width="0.8128" layer="51"/>
<wire x1="-11.43" y1="0" x2="-10.795" y2="0" width="0.8128" layer="51"/>
<wire x1="-10.16" y1="-4.191" x2="-10.16" y2="4.191" width="0.1524" layer="21"/>
<wire x1="-9.779" y1="4.572" x2="-8.89" y2="4.572" width="0.1524" layer="21"/>
<wire x1="-8.636" y1="4.318" x2="-8.89" y2="4.572" width="0.1524" layer="21"/>
<wire x1="-9.779" y1="-4.572" x2="-8.89" y2="-4.572" width="0.1524" layer="21"/>
<wire x1="-8.636" y1="-4.318" x2="-8.89" y2="-4.572" width="0.1524" layer="21"/>
<wire x1="8.636" y1="4.318" x2="8.89" y2="4.572" width="0.1524" layer="21"/>
<wire x1="8.636" y1="4.318" x2="-8.636" y2="4.318" width="0.1524" layer="21"/>
<wire x1="8.636" y1="-4.318" x2="8.89" y2="-4.572" width="0.1524" layer="21"/>
<wire x1="8.636" y1="-4.318" x2="-8.636" y2="-4.318" width="0.1524" layer="21"/>
<wire x1="9.779" y1="4.572" x2="8.89" y2="4.572" width="0.1524" layer="21"/>
<wire x1="9.779" y1="-4.572" x2="8.89" y2="-4.572" width="0.1524" layer="21"/>
<wire x1="10.16" y1="-4.191" x2="10.16" y2="4.191" width="0.1524" layer="21"/>
<wire x1="-10.16" y1="-4.191" x2="-9.779" y2="-4.572" width="0.1524" layer="21" curve="90"/>
<wire x1="-10.16" y1="4.191" x2="-9.779" y2="4.572" width="0.1524" layer="21" curve="-90"/>
<wire x1="9.779" y1="-4.572" x2="10.16" y2="-4.191" width="0.1524" layer="21" curve="90"/>
<wire x1="9.779" y1="4.572" x2="10.16" y2="4.191" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-11.43" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="11.43" y="0" drill="1.016" shape="octagon"/>
<text x="-10.16" y="5.1054" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-6.477" y="-0.5842" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-10.7188" y1="-0.4064" x2="-10.16" y2="0.4064" layer="51"/>
<rectangle x1="10.16" y1="-0.4064" x2="10.3124" y2="0.4064" layer="21"/>
<rectangle x1="-10.3124" y1="-0.4064" x2="-10.16" y2="0.4064" layer="21"/>
<rectangle x1="10.16" y1="-0.4064" x2="10.7188" y2="0.4064" layer="51"/>
</package>
<package name="P0613V">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0613, grid 5 mm</description>
<wire x1="2.54" y1="0" x2="1.397" y2="0" width="0.8128" layer="51"/>
<wire x1="-2.54" y1="0" x2="-1.397" y2="0" width="0.8128" layer="51"/>
<circle x="-2.54" y="0" radius="2.286" width="0.1524" layer="21"/>
<circle x="-2.54" y="0" radius="1.143" width="0.1524" layer="51"/>
<pad name="1" x="-2.54" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="1.016" shape="octagon"/>
<text x="-0.254" y="1.143" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-0.254" y="-2.413" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-1.2954" y1="-0.4064" x2="1.3208" y2="0.4064" layer="21"/>
</package>
<package name="P0613/15">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0613, grid 15 mm</description>
<wire x1="7.62" y1="0" x2="6.985" y2="0" width="0.8128" layer="51"/>
<wire x1="-7.62" y1="0" x2="-6.985" y2="0" width="0.8128" layer="51"/>
<wire x1="-6.477" y1="2.032" x2="-6.223" y2="2.286" width="0.1524" layer="21" curve="-90"/>
<wire x1="-6.477" y1="-2.032" x2="-6.223" y2="-2.286" width="0.1524" layer="21" curve="90"/>
<wire x1="6.223" y1="-2.286" x2="6.477" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="6.223" y1="2.286" x2="6.477" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<wire x1="-6.223" y1="2.286" x2="-5.334" y2="2.286" width="0.1524" layer="21"/>
<wire x1="-5.207" y1="2.159" x2="-5.334" y2="2.286" width="0.1524" layer="21"/>
<wire x1="-6.223" y1="-2.286" x2="-5.334" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="-5.207" y1="-2.159" x2="-5.334" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="5.207" y1="2.159" x2="5.334" y2="2.286" width="0.1524" layer="21"/>
<wire x1="5.207" y1="2.159" x2="-5.207" y2="2.159" width="0.1524" layer="21"/>
<wire x1="5.207" y1="-2.159" x2="5.334" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="5.207" y1="-2.159" x2="-5.207" y2="-2.159" width="0.1524" layer="21"/>
<wire x1="6.223" y1="2.286" x2="5.334" y2="2.286" width="0.1524" layer="21"/>
<wire x1="6.223" y1="-2.286" x2="5.334" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="6.477" y1="-0.635" x2="6.477" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="6.477" y1="-0.635" x2="6.477" y2="0.635" width="0.1524" layer="51"/>
<wire x1="6.477" y1="2.032" x2="6.477" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-6.477" y1="-2.032" x2="-6.477" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-6.477" y1="0.635" x2="-6.477" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="-6.477" y1="0.635" x2="-6.477" y2="2.032" width="0.1524" layer="21"/>
<pad name="1" x="-7.62" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="7.62" y="0" drill="1.016" shape="octagon"/>
<text x="-6.477" y="2.6924" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-4.318" y="-0.7112" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-7.0358" y1="-0.4064" x2="-6.477" y2="0.4064" layer="51"/>
<rectangle x1="6.477" y1="-0.4064" x2="7.0358" y2="0.4064" layer="51"/>
</package>
<package name="P0817/22">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0817, grid 22.5 mm</description>
<wire x1="-10.414" y1="0" x2="-11.43" y2="0" width="0.8128" layer="51"/>
<wire x1="-8.509" y1="-3.429" x2="-8.509" y2="3.429" width="0.1524" layer="21"/>
<wire x1="-8.128" y1="3.81" x2="-7.239" y2="3.81" width="0.1524" layer="21"/>
<wire x1="-6.985" y1="3.556" x2="-7.239" y2="3.81" width="0.1524" layer="21"/>
<wire x1="-8.128" y1="-3.81" x2="-7.239" y2="-3.81" width="0.1524" layer="21"/>
<wire x1="-6.985" y1="-3.556" x2="-7.239" y2="-3.81" width="0.1524" layer="21"/>
<wire x1="6.985" y1="3.556" x2="7.239" y2="3.81" width="0.1524" layer="21"/>
<wire x1="6.985" y1="3.556" x2="-6.985" y2="3.556" width="0.1524" layer="21"/>
<wire x1="6.985" y1="-3.556" x2="7.239" y2="-3.81" width="0.1524" layer="21"/>
<wire x1="6.985" y1="-3.556" x2="-6.985" y2="-3.556" width="0.1524" layer="21"/>
<wire x1="8.128" y1="3.81" x2="7.239" y2="3.81" width="0.1524" layer="21"/>
<wire x1="8.128" y1="-3.81" x2="7.239" y2="-3.81" width="0.1524" layer="21"/>
<wire x1="8.509" y1="-3.429" x2="8.509" y2="3.429" width="0.1524" layer="21"/>
<wire x1="11.43" y1="0" x2="10.414" y2="0" width="0.8128" layer="51"/>
<wire x1="-8.509" y1="3.429" x2="-8.128" y2="3.81" width="0.1524" layer="21" curve="-90"/>
<wire x1="-8.509" y1="-3.429" x2="-8.128" y2="-3.81" width="0.1524" layer="21" curve="90"/>
<wire x1="8.128" y1="3.81" x2="8.509" y2="3.429" width="0.1524" layer="21" curve="-90"/>
<wire x1="8.128" y1="-3.81" x2="8.509" y2="-3.429" width="0.1524" layer="21" curve="90"/>
<pad name="1" x="-11.43" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="11.43" y="0" drill="1.016" shape="octagon"/>
<text x="-8.382" y="4.2164" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-6.223" y="-0.5842" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="6.604" y="-2.2606" size="1.27" layer="51" ratio="10" rot="R90">0817</text>
<rectangle x1="8.509" y1="-0.4064" x2="10.3124" y2="0.4064" layer="21"/>
<rectangle x1="-10.3124" y1="-0.4064" x2="-8.509" y2="0.4064" layer="21"/>
</package>
<package name="P0817V">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0817, grid 6.35 mm</description>
<wire x1="-3.81" y1="0" x2="-5.08" y2="0" width="0.8128" layer="51"/>
<wire x1="1.27" y1="0" x2="0" y2="0" width="0.8128" layer="51"/>
<circle x="-5.08" y="0" radius="3.81" width="0.1524" layer="21"/>
<circle x="-5.08" y="0" radius="1.27" width="0.1524" layer="51"/>
<pad name="1" x="-5.08" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="1.016" shape="octagon"/>
<text x="-1.016" y="1.27" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.016" y="-2.54" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="-6.858" y="2.032" size="1.016" layer="21" ratio="12">0817</text>
<rectangle x1="-3.81" y1="-0.4064" x2="0" y2="0.4064" layer="21"/>
</package>
<package name="0922V">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0922, grid 7.5 mm</description>
<wire x1="2.54" y1="0" x2="1.397" y2="0" width="0.8128" layer="51"/>
<wire x1="-5.08" y1="0" x2="-3.81" y2="0" width="0.8128" layer="51"/>
<circle x="-5.08" y="0" radius="4.572" width="0.1524" layer="21"/>
<circle x="-5.08" y="0" radius="1.905" width="0.1524" layer="21"/>
<pad name="1" x="-5.08" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="1.016" shape="octagon"/>
<text x="-0.508" y="1.6764" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-0.508" y="-2.9972" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="-6.858" y="2.54" size="1.016" layer="21" ratio="12">0922</text>
<rectangle x1="-3.81" y1="-0.4064" x2="1.3208" y2="0.4064" layer="21"/>
</package>
<package name="MINI_MELF-0204R">
<description>&lt;b&gt;CECC Size RC3715&lt;/b&gt; Reflow Soldering&lt;p&gt;
source Beyschlag</description>
<wire x1="-1.7" y1="-0.6" x2="1.7" y2="-0.6" width="0.2032" layer="51"/>
<wire x1="1.7" y1="-0.6" x2="1.7" y2="0.6" width="0.2032" layer="51"/>
<wire x1="1.7" y1="0.6" x2="-1.7" y2="0.6" width="0.2032" layer="51"/>
<wire x1="-1.7" y1="0.6" x2="-1.7" y2="-0.6" width="0.2032" layer="51"/>
<wire x1="0.938" y1="0.6" x2="-0.938" y2="0.6" width="0.2032" layer="21"/>
<wire x1="-0.938" y1="-0.6" x2="0.938" y2="-0.6" width="0.2032" layer="21"/>
<smd name="1" x="-1.5" y="0" dx="0.8" dy="1.6" layer="1"/>
<smd name="2" x="1.5" y="0" dx="0.8" dy="1.6" layer="1"/>
<text x="-1.27" y="0.9525" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.2225" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="MINI_MELF-0204W">
<description>&lt;b&gt;CECC Size RC3715&lt;/b&gt; Wave Soldering&lt;p&gt;
source Beyschlag</description>
<wire x1="-1.7" y1="-0.6" x2="1.7" y2="-0.6" width="0.2032" layer="51"/>
<wire x1="1.7" y1="-0.6" x2="1.7" y2="0.6" width="0.2032" layer="51"/>
<wire x1="1.7" y1="0.6" x2="-1.7" y2="0.6" width="0.2032" layer="51"/>
<wire x1="-1.7" y1="0.6" x2="-1.7" y2="-0.6" width="0.2032" layer="51"/>
<wire x1="0.684" y1="0.6" x2="-0.684" y2="0.6" width="0.2032" layer="21"/>
<wire x1="-0.684" y1="-0.6" x2="0.684" y2="-0.6" width="0.2032" layer="21"/>
<smd name="1" x="-1.5" y="0" dx="1.2" dy="1.6" layer="1"/>
<smd name="2" x="1.5" y="0" dx="1.2" dy="1.6" layer="1"/>
<text x="-1.27" y="0.9525" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.2225" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="MINI_MELF-0207R">
<description>&lt;b&gt;CECC Size RC6123&lt;/b&gt; Reflow Soldering&lt;p&gt;
source Beyschlag</description>
<wire x1="-2.8" y1="-1" x2="2.8" y2="-1" width="0.2032" layer="51"/>
<wire x1="2.8" y1="-1" x2="2.8" y2="1" width="0.2032" layer="51"/>
<wire x1="2.8" y1="1" x2="-2.8" y2="1" width="0.2032" layer="51"/>
<wire x1="-2.8" y1="1" x2="-2.8" y2="-1" width="0.2032" layer="51"/>
<wire x1="1.2125" y1="1" x2="-1.2125" y2="1" width="0.2032" layer="21"/>
<wire x1="-1.2125" y1="-1" x2="1.2125" y2="-1" width="0.2032" layer="21"/>
<smd name="1" x="-2.25" y="0" dx="1.6" dy="2.5" layer="1"/>
<smd name="2" x="2.25" y="0" dx="1.6" dy="2.5" layer="1"/>
<text x="-2.2225" y="1.5875" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.2225" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="MINI_MELF-0207W">
<description>&lt;b&gt;CECC Size RC6123&lt;/b&gt; Wave Soldering&lt;p&gt;
source Beyschlag</description>
<wire x1="-2.8" y1="-1" x2="2.8" y2="-1" width="0.2032" layer="51"/>
<wire x1="2.8" y1="-1" x2="2.8" y2="1" width="0.2032" layer="51"/>
<wire x1="2.8" y1="1" x2="-2.8" y2="1" width="0.2032" layer="51"/>
<wire x1="-2.8" y1="1" x2="-2.8" y2="-1" width="0.2032" layer="51"/>
<wire x1="1.149" y1="1" x2="-1.149" y2="1" width="0.2032" layer="21"/>
<wire x1="-1.149" y1="-1" x2="1.149" y2="-1" width="0.2032" layer="21"/>
<smd name="1" x="-2.6" y="0" dx="2.4" dy="2.5" layer="1"/>
<smd name="2" x="2.6" y="0" dx="2.4" dy="2.5" layer="1"/>
<text x="-2.54" y="1.5875" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="0309V">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0309, grid 2.5 mm</description>
<wire x1="1.27" y1="0" x2="0.635" y2="0" width="0.6096" layer="51"/>
<wire x1="-0.635" y1="0" x2="-1.27" y2="0" width="0.6096" layer="51"/>
<circle x="-1.27" y="0" radius="1.524" width="0.1524" layer="21"/>
<circle x="-1.27" y="0" radius="0.762" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="0.254" y="1.016" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="0.254" y="-2.2098" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="0.254" y1="-0.3048" x2="0.5588" y2="0.3048" layer="51"/>
<rectangle x1="-0.635" y1="-0.3048" x2="-0.3302" y2="0.3048" layer="51"/>
<rectangle x1="-0.3302" y1="-0.3048" x2="0.254" y2="0.3048" layer="21"/>
</package>
<package name="VMTA55">
<description>&lt;b&gt;Bulk Metal® Foil Technology&lt;/b&gt;, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements&lt;p&gt;
MIL SIZE RNC55&lt;br&gt;
Source: VISHAY .. vta56.pdf</description>
<wire x1="-5.08" y1="0" x2="-4.26" y2="0" width="0.6096" layer="51"/>
<wire x1="3.3375" y1="-1.45" x2="3.3375" y2="1.45" width="0.1524" layer="21"/>
<wire x1="3.3375" y1="1.45" x2="-3.3625" y2="1.45" width="0.1524" layer="21"/>
<wire x1="-3.3625" y1="1.45" x2="-3.3625" y2="-1.45" width="0.1524" layer="21"/>
<wire x1="-3.3625" y1="-1.45" x2="3.3375" y2="-1.45" width="0.1524" layer="21"/>
<wire x1="4.235" y1="0" x2="5.08" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-5.08" y="0" drill="1.1" shape="octagon"/>
<pad name="2" x="5.08" y="0" drill="1.1" shape="octagon"/>
<text x="-3.175" y="1.905" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-4.26" y1="-0.3048" x2="-3.3075" y2="0.3048" layer="21"/>
<rectangle x1="3.2825" y1="-0.3048" x2="4.235" y2="0.3048" layer="21"/>
</package>
<package name="VMTB60">
<description>&lt;b&gt;Bulk Metal® Foil Technology&lt;/b&gt;, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements&lt;p&gt;
MIL SIZE RNC60&lt;br&gt;
Source: VISHAY .. vta56.pdf</description>
<wire x1="-6.35" y1="0" x2="-5.585" y2="0" width="0.6096" layer="51"/>
<wire x1="4.6875" y1="-1.95" x2="4.6875" y2="1.95" width="0.1524" layer="21"/>
<wire x1="4.6875" y1="1.95" x2="-4.6875" y2="1.95" width="0.1524" layer="21"/>
<wire x1="-4.6875" y1="1.95" x2="-4.6875" y2="-1.95" width="0.1524" layer="21"/>
<wire x1="-4.6875" y1="-1.95" x2="4.6875" y2="-1.95" width="0.1524" layer="21"/>
<wire x1="5.585" y1="0" x2="6.35" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-6.35" y="0" drill="1.1" shape="octagon"/>
<pad name="2" x="6.35" y="0" drill="1.1" shape="octagon"/>
<text x="-4.445" y="2.54" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-4.445" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-5.585" y1="-0.3048" x2="-4.6325" y2="0.3048" layer="21"/>
<rectangle x1="4.6325" y1="-0.3048" x2="5.585" y2="0.3048" layer="21"/>
</package>
<package name="VTA52">
<description>&lt;b&gt;Bulk Metal® Foil Technology&lt;/b&gt;, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements&lt;p&gt;
MIL SIZE RBR52&lt;br&gt;
Source: VISHAY .. vta56.pdf</description>
<wire x1="-15.24" y1="0" x2="-13.97" y2="0" width="0.6096" layer="51"/>
<wire x1="12.6225" y1="0.025" x2="12.6225" y2="4.725" width="0.1524" layer="21"/>
<wire x1="12.6225" y1="4.725" x2="-12.6225" y2="4.725" width="0.1524" layer="21"/>
<wire x1="-12.6225" y1="4.725" x2="-12.6225" y2="0.025" width="0.1524" layer="21"/>
<wire x1="-12.6225" y1="0.025" x2="-12.6225" y2="-4.65" width="0.1524" layer="21"/>
<wire x1="-12.6225" y1="-4.65" x2="12.6225" y2="-4.65" width="0.1524" layer="21"/>
<wire x1="12.6225" y1="-4.65" x2="12.6225" y2="0.025" width="0.1524" layer="21"/>
<wire x1="13.97" y1="0" x2="15.24" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-15.24" y="0" drill="1.1" shape="octagon"/>
<pad name="2" x="15.24" y="0" drill="1.1" shape="octagon"/>
<text x="-3.81" y="5.08" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-13.97" y1="-0.3048" x2="-12.5675" y2="0.3048" layer="21"/>
<rectangle x1="12.5675" y1="-0.3048" x2="13.97" y2="0.3048" layer="21"/>
</package>
<package name="VTA53">
<description>&lt;b&gt;Bulk Metal® Foil Technology&lt;/b&gt;, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements&lt;p&gt;
MIL SIZE RBR53&lt;br&gt;
Source: VISHAY .. vta56.pdf</description>
<wire x1="-12.065" y1="0" x2="-10.795" y2="0" width="0.6096" layer="51"/>
<wire x1="9.8975" y1="0" x2="9.8975" y2="4.7" width="0.1524" layer="21"/>
<wire x1="9.8975" y1="4.7" x2="-9.8975" y2="4.7" width="0.1524" layer="21"/>
<wire x1="-9.8975" y1="4.7" x2="-9.8975" y2="0" width="0.1524" layer="21"/>
<wire x1="-9.8975" y1="0" x2="-9.8975" y2="-4.675" width="0.1524" layer="21"/>
<wire x1="-9.8975" y1="-4.675" x2="9.8975" y2="-4.675" width="0.1524" layer="21"/>
<wire x1="9.8975" y1="-4.675" x2="9.8975" y2="0" width="0.1524" layer="21"/>
<wire x1="10.795" y1="0" x2="12.065" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-12.065" y="0" drill="1.1" shape="octagon"/>
<pad name="2" x="12.065" y="0" drill="1.1" shape="octagon"/>
<text x="-3.81" y="5.08" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-10.795" y1="-0.3048" x2="-9.8425" y2="0.3048" layer="21"/>
<rectangle x1="9.8425" y1="-0.3048" x2="10.795" y2="0.3048" layer="21"/>
</package>
<package name="VTA54">
<description>&lt;b&gt;Bulk Metal® Foil Technology&lt;/b&gt;, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements&lt;p&gt;
MIL SIZE RBR54&lt;br&gt;
Source: VISHAY .. vta56.pdf</description>
<wire x1="-12.065" y1="0" x2="-10.795" y2="0" width="0.6096" layer="51"/>
<wire x1="9.8975" y1="0" x2="9.8975" y2="3.3" width="0.1524" layer="21"/>
<wire x1="9.8975" y1="3.3" x2="-9.8975" y2="3.3" width="0.1524" layer="21"/>
<wire x1="-9.8975" y1="3.3" x2="-9.8975" y2="0" width="0.1524" layer="21"/>
<wire x1="-9.8975" y1="0" x2="-9.8975" y2="-3.3" width="0.1524" layer="21"/>
<wire x1="-9.8975" y1="-3.3" x2="9.8975" y2="-3.3" width="0.1524" layer="21"/>
<wire x1="9.8975" y1="-3.3" x2="9.8975" y2="0" width="0.1524" layer="21"/>
<wire x1="10.795" y1="0" x2="12.065" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-12.065" y="0" drill="1.1" shape="octagon"/>
<pad name="2" x="12.065" y="0" drill="1.1" shape="octagon"/>
<text x="-3.81" y="3.81" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-10.795" y1="-0.3048" x2="-9.8425" y2="0.3048" layer="21"/>
<rectangle x1="9.8425" y1="-0.3048" x2="10.795" y2="0.3048" layer="21"/>
</package>
<package name="VTA55">
<description>&lt;b&gt;Bulk Metal® Foil Technology&lt;/b&gt;, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements&lt;p&gt;
MIL SIZE RBR55&lt;br&gt;
Source: VISHAY .. vta56.pdf</description>
<wire x1="-8.255" y1="0" x2="-6.985" y2="0" width="0.6096" layer="51"/>
<wire x1="6.405" y1="0" x2="6.405" y2="3.3" width="0.1524" layer="21"/>
<wire x1="6.405" y1="3.3" x2="-6.405" y2="3.3" width="0.1524" layer="21"/>
<wire x1="-6.405" y1="3.3" x2="-6.405" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.405" y1="0" x2="-6.405" y2="-3.3" width="0.1524" layer="21"/>
<wire x1="-6.405" y1="-3.3" x2="6.405" y2="-3.3" width="0.1524" layer="21"/>
<wire x1="6.405" y1="-3.3" x2="6.405" y2="0" width="0.1524" layer="21"/>
<wire x1="6.985" y1="0" x2="8.255" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-8.255" y="0" drill="1.1" shape="octagon"/>
<pad name="2" x="8.255" y="0" drill="1.1" shape="octagon"/>
<text x="-3.81" y="3.81" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-6.985" y1="-0.3048" x2="-6.35" y2="0.3048" layer="21"/>
<rectangle x1="6.35" y1="-0.3048" x2="6.985" y2="0.3048" layer="21"/>
</package>
<package name="VTA56">
<description>&lt;b&gt;Bulk Metal® Foil Technology&lt;/b&gt;, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements&lt;p&gt;
MIL SIZE RBR56&lt;br&gt;
Source: VISHAY .. vta56.pdf</description>
<wire x1="-6.35" y1="0" x2="-5.08" y2="0" width="0.6096" layer="51"/>
<wire x1="4.5" y1="0" x2="4.5" y2="3.3" width="0.1524" layer="21"/>
<wire x1="4.5" y1="3.3" x2="-4.5" y2="3.3" width="0.1524" layer="21"/>
<wire x1="-4.5" y1="3.3" x2="-4.5" y2="0" width="0.1524" layer="21"/>
<wire x1="-4.5" y1="0" x2="-4.5" y2="-3.3" width="0.1524" layer="21"/>
<wire x1="-4.5" y1="-3.3" x2="4.5" y2="-3.3" width="0.1524" layer="21"/>
<wire x1="4.5" y1="-3.3" x2="4.5" y2="0" width="0.1524" layer="21"/>
<wire x1="5.08" y1="0" x2="6.35" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-6.35" y="0" drill="1.1" shape="octagon"/>
<pad name="2" x="6.35" y="0" drill="1.1" shape="octagon"/>
<text x="-3.81" y="3.81" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-5.08" y1="-0.3048" x2="-4.445" y2="0.3048" layer="21"/>
<rectangle x1="4.445" y1="-0.3048" x2="5.08" y2="0.3048" layer="21"/>
</package>
<package name="R4527">
<description>&lt;b&gt;Package 4527&lt;/b&gt;&lt;p&gt;
Source: http://www.vishay.com/docs/31059/wsrhigh.pdf</description>
<wire x1="-5.675" y1="-3.375" x2="5.65" y2="-3.375" width="0.2032" layer="21"/>
<wire x1="5.65" y1="-3.375" x2="5.65" y2="3.375" width="0.2032" layer="51"/>
<wire x1="5.65" y1="3.375" x2="-5.675" y2="3.375" width="0.2032" layer="21"/>
<wire x1="-5.675" y1="3.375" x2="-5.675" y2="-3.375" width="0.2032" layer="51"/>
<smd name="1" x="-4.575" y="0" dx="3.94" dy="5.84" layer="1"/>
<smd name="2" x="4.575" y="0" dx="3.94" dy="5.84" layer="1"/>
<text x="-5.715" y="3.81" size="1.27" layer="25">&gt;NAME</text>
<text x="-5.715" y="-5.08" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="WSC0001">
<description>&lt;b&gt;Wirewound Resistors, Precision Power&lt;/b&gt;&lt;p&gt;
Source: VISHAY wscwsn.pdf</description>
<wire x1="-3.075" y1="1.8" x2="-3.075" y2="-1.8" width="0.2032" layer="51"/>
<wire x1="-3.075" y1="-1.8" x2="3.075" y2="-1.8" width="0.2032" layer="21"/>
<wire x1="3.075" y1="-1.8" x2="3.075" y2="1.8" width="0.2032" layer="51"/>
<wire x1="3.075" y1="1.8" x2="-3.075" y2="1.8" width="0.2032" layer="21"/>
<wire x1="-3.075" y1="1.8" x2="-3.075" y2="1.606" width="0.2032" layer="21"/>
<wire x1="-3.075" y1="-1.606" x2="-3.075" y2="-1.8" width="0.2032" layer="21"/>
<wire x1="3.075" y1="1.606" x2="3.075" y2="1.8" width="0.2032" layer="21"/>
<wire x1="3.075" y1="-1.8" x2="3.075" y2="-1.606" width="0.2032" layer="21"/>
<smd name="1" x="-2.675" y="0" dx="2.29" dy="2.92" layer="1"/>
<smd name="2" x="2.675" y="0" dx="2.29" dy="2.92" layer="1"/>
<text x="-2.544" y="2.229" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.544" y="-3.501" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="WSC0002">
<description>&lt;b&gt;Wirewound Resistors, Precision Power&lt;/b&gt;&lt;p&gt;
Source: VISHAY wscwsn.pdf</description>
<wire x1="-5.55" y1="3.375" x2="-5.55" y2="-3.375" width="0.2032" layer="51"/>
<wire x1="-5.55" y1="-3.375" x2="5.55" y2="-3.375" width="0.2032" layer="21"/>
<wire x1="5.55" y1="-3.375" x2="5.55" y2="3.375" width="0.2032" layer="51"/>
<wire x1="5.55" y1="3.375" x2="-5.55" y2="3.375" width="0.2032" layer="21"/>
<smd name="1" x="-4.575" y="0.025" dx="3.94" dy="5.84" layer="1"/>
<smd name="2" x="4.575" y="0" dx="3.94" dy="5.84" layer="1"/>
<text x="-5.65" y="3.9" size="1.27" layer="25">&gt;NAME</text>
<text x="-5.65" y="-5.15" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="WSC01/2">
<description>&lt;b&gt;Wirewound Resistors, Precision Power&lt;/b&gt;&lt;p&gt;
Source: VISHAY wscwsn.pdf</description>
<wire x1="-2.45" y1="1.475" x2="-2.45" y2="-1.475" width="0.2032" layer="51"/>
<wire x1="-2.45" y1="-1.475" x2="2.45" y2="-1.475" width="0.2032" layer="21"/>
<wire x1="2.45" y1="-1.475" x2="2.45" y2="1.475" width="0.2032" layer="51"/>
<wire x1="2.45" y1="1.475" x2="-2.45" y2="1.475" width="0.2032" layer="21"/>
<wire x1="-2.45" y1="1.475" x2="-2.45" y2="1.106" width="0.2032" layer="21"/>
<wire x1="-2.45" y1="-1.106" x2="-2.45" y2="-1.475" width="0.2032" layer="21"/>
<wire x1="2.45" y1="1.106" x2="2.45" y2="1.475" width="0.2032" layer="21"/>
<wire x1="2.45" y1="-1.475" x2="2.45" y2="-1.106" width="0.2032" layer="21"/>
<smd name="1" x="-2.1" y="0" dx="2.16" dy="1.78" layer="1"/>
<smd name="2" x="2.1" y="0" dx="2.16" dy="1.78" layer="1"/>
<text x="-2.544" y="1.904" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.544" y="-3.176" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="WSC2515">
<description>&lt;b&gt;Wirewound Resistors, Precision Power&lt;/b&gt;&lt;p&gt;
Source: VISHAY wscwsn.pdf</description>
<wire x1="-3.075" y1="1.8" x2="-3.075" y2="-1.8" width="0.2032" layer="51"/>
<wire x1="-3.075" y1="-1.8" x2="3.05" y2="-1.8" width="0.2032" layer="21"/>
<wire x1="3.05" y1="-1.8" x2="3.05" y2="1.8" width="0.2032" layer="51"/>
<wire x1="3.05" y1="1.8" x2="-3.075" y2="1.8" width="0.2032" layer="21"/>
<wire x1="-3.075" y1="1.8" x2="-3.075" y2="1.606" width="0.2032" layer="21"/>
<wire x1="-3.075" y1="-1.606" x2="-3.075" y2="-1.8" width="0.2032" layer="21"/>
<wire x1="3.05" y1="1.606" x2="3.05" y2="1.8" width="0.2032" layer="21"/>
<wire x1="3.05" y1="-1.8" x2="3.05" y2="-1.606" width="0.2032" layer="21"/>
<smd name="1" x="-2.675" y="0" dx="2.29" dy="2.92" layer="1"/>
<smd name="2" x="2.675" y="0" dx="2.29" dy="2.92" layer="1"/>
<text x="-3.2" y="2.15" size="1.27" layer="25">&gt;NAME</text>
<text x="-3.2" y="-3.4" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="WSC4527">
<description>&lt;b&gt;Wirewound Resistors, Precision Power&lt;/b&gt;&lt;p&gt;
Source: VISHAY wscwsn.pdf</description>
<wire x1="-5.675" y1="3.4" x2="-5.675" y2="-3.375" width="0.2032" layer="51"/>
<wire x1="-5.675" y1="-3.375" x2="5.675" y2="-3.375" width="0.2032" layer="21"/>
<wire x1="5.675" y1="-3.375" x2="5.675" y2="3.4" width="0.2032" layer="51"/>
<wire x1="5.675" y1="3.4" x2="-5.675" y2="3.4" width="0.2032" layer="21"/>
<smd name="1" x="-4.575" y="0.025" dx="3.94" dy="5.84" layer="1"/>
<smd name="2" x="4.575" y="0" dx="3.94" dy="5.84" layer="1"/>
<text x="-5.775" y="3.925" size="1.27" layer="25">&gt;NAME</text>
<text x="-5.775" y="-5.15" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="WSC6927">
<description>&lt;b&gt;Wirewound Resistors, Precision Power&lt;/b&gt;&lt;p&gt;
Source: VISHAY wscwsn.pdf</description>
<wire x1="-8.65" y1="3.375" x2="-8.65" y2="-3.375" width="0.2032" layer="51"/>
<wire x1="-8.65" y1="-3.375" x2="8.65" y2="-3.375" width="0.2032" layer="21"/>
<wire x1="8.65" y1="-3.375" x2="8.65" y2="3.375" width="0.2032" layer="51"/>
<wire x1="8.65" y1="3.375" x2="-8.65" y2="3.375" width="0.2032" layer="21"/>
<smd name="1" x="-7.95" y="0.025" dx="3.94" dy="5.97" layer="1"/>
<smd name="2" x="7.95" y="0" dx="3.94" dy="5.97" layer="1"/>
<text x="-8.75" y="3.9" size="1.27" layer="25">&gt;NAME</text>
<text x="-8.75" y="-5.15" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="R1218">
<description>&lt;b&gt;CRCW1218 Thick Film, Rectangular Chip Resistors&lt;/b&gt;&lt;p&gt;
Source: http://www.vishay.com .. dcrcw.pdf</description>
<wire x1="-0.913" y1="-2.219" x2="0.939" y2="-2.219" width="0.1524" layer="51"/>
<wire x1="0.913" y1="2.219" x2="-0.939" y2="2.219" width="0.1524" layer="51"/>
<smd name="1" x="-1.475" y="0" dx="1.05" dy="4.9" layer="1"/>
<smd name="2" x="1.475" y="0" dx="1.05" dy="4.9" layer="1"/>
<text x="-2.54" y="2.54" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-3.81" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.651" y1="-2.3" x2="-0.9009" y2="2.3" layer="51"/>
<rectangle x1="0.9144" y1="-2.3" x2="1.6645" y2="2.3" layer="51"/>
</package>
<package name="R0805W">
<description>&lt;b&gt;RESISTOR&lt;/b&gt; wave soldering&lt;p&gt;</description>
<wire x1="-0.41" y1="0.635" x2="0.41" y2="0.635" width="0.1524" layer="51"/>
<wire x1="-0.41" y1="-0.635" x2="0.41" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="-1.973" y1="0.983" x2="1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="0.983" x2="1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="-0.983" x2="-1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.973" y1="-0.983" x2="-1.973" y2="0.983" width="0.0508" layer="39"/>
<smd name="1" x="-1.0525" y="0" dx="1.5" dy="1" layer="1"/>
<smd name="2" x="1.0525" y="0" dx="1.5" dy="1" layer="1"/>
<text x="-0.635" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="0.4064" y1="-0.6985" x2="1.0564" y2="0.7015" layer="51"/>
<rectangle x1="-1.0668" y1="-0.6985" x2="-0.4168" y2="0.7015" layer="51"/>
<rectangle x1="-0.1999" y1="-0.5001" x2="0.1999" y2="0.5001" layer="35"/>
</package>
<package name="0204/5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0204, grid 5 mm</description>
<wire x1="2.54" y1="0" x2="2.032" y2="0" width="0.508" layer="51"/>
<wire x1="-2.54" y1="0" x2="-2.032" y2="0" width="0.508" layer="51"/>
<wire x1="-1.778" y1="0.635" x2="-1.524" y2="0.889" width="0.1524" layer="21" curve="-90"/>
<wire x1="-1.778" y1="-0.635" x2="-1.524" y2="-0.889" width="0.1524" layer="21" curve="90"/>
<wire x1="1.524" y1="-0.889" x2="1.778" y2="-0.635" width="0.1524" layer="21" curve="90"/>
<wire x1="1.524" y1="0.889" x2="1.778" y2="0.635" width="0.1524" layer="21" curve="-90"/>
<wire x1="-1.778" y1="-0.635" x2="-1.778" y2="0.635" width="0.1524" layer="51"/>
<wire x1="-1.524" y1="0.889" x2="-1.27" y2="0.889" width="0.1524" layer="21"/>
<wire x1="-1.143" y1="0.762" x2="-1.27" y2="0.889" width="0.1524" layer="21"/>
<wire x1="-1.524" y1="-0.889" x2="-1.27" y2="-0.889" width="0.1524" layer="21"/>
<wire x1="-1.143" y1="-0.762" x2="-1.27" y2="-0.889" width="0.1524" layer="21"/>
<wire x1="1.143" y1="0.762" x2="1.27" y2="0.889" width="0.1524" layer="21"/>
<wire x1="1.143" y1="0.762" x2="-1.143" y2="0.762" width="0.1524" layer="21"/>
<wire x1="1.143" y1="-0.762" x2="1.27" y2="-0.889" width="0.1524" layer="21"/>
<wire x1="1.143" y1="-0.762" x2="-1.143" y2="-0.762" width="0.1524" layer="21"/>
<wire x1="1.524" y1="0.889" x2="1.27" y2="0.889" width="0.1524" layer="21"/>
<wire x1="1.524" y1="-0.889" x2="1.27" y2="-0.889" width="0.1524" layer="21"/>
<wire x1="1.778" y1="-0.635" x2="1.778" y2="0.635" width="0.1524" layer="51"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.0066" y="1.1684" size="0.9906" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.1336" y="-2.3114" size="0.9906" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-2.032" y1="-0.254" x2="-1.778" y2="0.254" layer="51"/>
<rectangle x1="1.778" y1="-0.254" x2="2.032" y2="0.254" layer="51"/>
</package>
<package name="0204/7">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0204, grid 7.5 mm</description>
<wire x1="3.81" y1="0" x2="2.921" y2="0" width="0.508" layer="51"/>
<wire x1="-3.81" y1="0" x2="-2.921" y2="0" width="0.508" layer="51"/>
<wire x1="-2.54" y1="0.762" x2="-2.286" y2="1.016" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.54" y1="-0.762" x2="-2.286" y2="-1.016" width="0.1524" layer="21" curve="90"/>
<wire x1="2.286" y1="-1.016" x2="2.54" y2="-0.762" width="0.1524" layer="21" curve="90"/>
<wire x1="2.286" y1="1.016" x2="2.54" y2="0.762" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.54" y1="-0.762" x2="-2.54" y2="0.762" width="0.1524" layer="21"/>
<wire x1="-2.286" y1="1.016" x2="-1.905" y2="1.016" width="0.1524" layer="21"/>
<wire x1="-1.778" y1="0.889" x2="-1.905" y2="1.016" width="0.1524" layer="21"/>
<wire x1="-2.286" y1="-1.016" x2="-1.905" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-1.778" y1="-0.889" x2="-1.905" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="1.778" y1="0.889" x2="1.905" y2="1.016" width="0.1524" layer="21"/>
<wire x1="1.778" y1="0.889" x2="-1.778" y2="0.889" width="0.1524" layer="21"/>
<wire x1="1.778" y1="-0.889" x2="1.905" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="1.778" y1="-0.889" x2="-1.778" y2="-0.889" width="0.1524" layer="21"/>
<wire x1="2.286" y1="1.016" x2="1.905" y2="1.016" width="0.1524" layer="21"/>
<wire x1="2.286" y1="-1.016" x2="1.905" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="2.54" y1="-0.762" x2="2.54" y2="0.762" width="0.1524" layer="21"/>
<pad name="1" x="-3.81" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="3.81" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.54" y="1.2954" size="0.9906" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.6256" y="-0.4826" size="0.9906" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="2.54" y1="-0.254" x2="2.921" y2="0.254" layer="21"/>
<rectangle x1="-2.921" y1="-0.254" x2="-2.54" y2="0.254" layer="21"/>
</package>
<package name="0204V">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0204, grid 2.5 mm</description>
<wire x1="-1.27" y1="0" x2="1.27" y2="0" width="0.508" layer="51"/>
<wire x1="-0.127" y1="0" x2="0.127" y2="0" width="0.508" layer="21"/>
<circle x="-1.27" y="0" radius="0.889" width="0.1524" layer="51"/>
<circle x="-1.27" y="0" radius="0.635" width="0.0508" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.1336" y="1.1684" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.1336" y="-2.3114" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="1812X7R">
<description>&lt;b&gt;Chip Monolithic Ceramic Capacitors&lt;/b&gt; Medium Voltage High Capacitance for General Use&lt;p&gt;
Source: http://www.murata.com .. GRM43DR72E224KW01.pdf</description>
<wire x1="-1.1" y1="1.5" x2="1.1" y2="1.5" width="0.2032" layer="51"/>
<wire x1="1.1" y1="-1.5" x2="-1.1" y2="-1.5" width="0.2032" layer="51"/>
<wire x1="-0.6" y1="1.5" x2="0.6" y2="1.5" width="0.2032" layer="21"/>
<wire x1="0.6" y1="-1.5" x2="-0.6" y2="-1.5" width="0.2032" layer="21"/>
<smd name="1" x="-1.425" y="0" dx="0.8" dy="3.5" layer="1"/>
<smd name="2" x="1.425" y="0" dx="0.8" dy="3.5" layer="1" rot="R180"/>
<text x="-1.9456" y="1.9958" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.9456" y="-3.7738" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.4" y1="-1.6" x2="-1.1" y2="1.6" layer="51"/>
<rectangle x1="1.1" y1="-1.6" x2="1.4" y2="1.6" layer="51" rot="R180"/>
</package>
<package name="R01005">
<smd name="1" x="-0.1625" y="0" dx="0.2" dy="0.25" layer="1"/>
<smd name="2" x="0.1625" y="0" dx="0.2" dy="0.25" layer="1"/>
<text x="-0.4" y="0.3" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.4" y="-1.6" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.2" y1="-0.1" x2="-0.075" y2="0.1" layer="51"/>
<rectangle x1="0.075" y1="-0.1" x2="0.2" y2="0.1" layer="51"/>
<rectangle x1="-0.15" y1="0.05" x2="0.15" y2="0.1" layer="51"/>
<rectangle x1="-0.15" y1="-0.1" x2="0.15" y2="-0.05" layer="51"/>
</package>
</packages>
<symbols>
<symbol name="R-US">
<wire x1="-2.54" y1="0" x2="-2.159" y2="1.016" width="0.2032" layer="94"/>
<wire x1="-2.159" y1="1.016" x2="-1.524" y2="-1.016" width="0.2032" layer="94"/>
<wire x1="-1.524" y1="-1.016" x2="-0.889" y2="1.016" width="0.2032" layer="94"/>
<wire x1="-0.889" y1="1.016" x2="-0.254" y2="-1.016" width="0.2032" layer="94"/>
<wire x1="-0.254" y1="-1.016" x2="0.381" y2="1.016" width="0.2032" layer="94"/>
<wire x1="0.381" y1="1.016" x2="1.016" y2="-1.016" width="0.2032" layer="94"/>
<wire x1="1.016" y1="-1.016" x2="1.651" y2="1.016" width="0.2032" layer="94"/>
<wire x1="1.651" y1="1.016" x2="2.286" y2="-1.016" width="0.2032" layer="94"/>
<wire x1="2.286" y1="-1.016" x2="2.54" y2="0" width="0.2032" layer="94"/>
<text x="-3.81" y="1.4986" size="1.778" layer="95">&gt;NAME</text>
<text x="-3.81" y="-3.302" size="1.778" layer="96">&gt;VALUE</text>
<pin name="2" x="5.08" y="0" visible="off" length="short" direction="pas" swaplevel="1" rot="R180"/>
<pin name="1" x="-5.08" y="0" visible="off" length="short" direction="pas" swaplevel="1"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="R-US_" prefix="R" uservalue="yes">
<description>&lt;B&gt;RESISTOR&lt;/B&gt;, American symbol</description>
<gates>
<gate name="G$1" symbol="R-US" x="0" y="0"/>
</gates>
<devices>
<device name="R0402" package="R0402">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R0603" package="R0603">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R0805" package="R0805">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R0805W" package="R0805W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R1206" package="R1206">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R1206W" package="R1206W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R1210" package="R1210">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R1210W" package="R1210W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R2010" package="R2010">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R2010W" package="R2010W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R2012" package="R2012">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R2012W" package="R2012W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R2512" package="R2512">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R2512W" package="R2512W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R3216" package="R3216">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R3216W" package="R3216W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R3225" package="R3225">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R3225W" package="R3225W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R5025" package="R5025">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R5025W" package="R5025W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R6332" package="R6332">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R6332W" package="R6332W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="M0805" package="M0805">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="M1206" package="M1206">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="M1406" package="M1406">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="M2012" package="M2012">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="M2309" package="M2309">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="M3216" package="M3216">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="M3516" package="M3516">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="M5923" package="M5923">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0204/5" package="0204/5">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0204/7" package="0204/7">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0207/10" package="0207/10">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0207/12" package="0207/12">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0207/15" package="0207/15">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0207/2V" package="0207/2V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0207/5V" package="0207/5V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0207/7" package="0207/7">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0309/10" package="0309/10">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0309/12" package="0309/12">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0411/12" package="0411/12">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0411/15" package="0411/15">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0411/3V" package="0411V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0414/15" package="0414/15">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0414/5V" package="0414V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0617/17" package="0617/17">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0617/22" package="0617/22">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0617/5V" package="0617V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0922/22" package="0922/22">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0613/5V" package="P0613V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0613/15" package="P0613/15">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0817/22" package="P0817/22">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0817/7V" package="P0817V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="V234/12" package="V234/12">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="V235/17" package="V235/17">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="V526-0" package="V526-0">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="MELF0102AX" package="MINI_MELF-0102AX">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0922V" package="0922V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="MELF0102R" package="MINI_MELF-0102R">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="MELF0102W" package="MINI_MELF-0102W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="MELF0204R" package="MINI_MELF-0204R">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="MELF0204W" package="MINI_MELF-0204W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="MELF0207R" package="MINI_MELF-0207R">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="MELF0207W" package="MINI_MELF-0207W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="RDH/15" package="RDH/15">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0204/2V" package="0204V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0309/V" package="0309V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R0201" package="R0201">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="VMTA55" package="VMTA55">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="VMTB60" package="VMTB60">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="VTA52" package="VTA52">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="VTA53" package="VTA53">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="VTA54" package="VTA54">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="VTA55" package="VTA55">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="VTA56" package="VTA56">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R4527" package="R4527">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="WSC0001" package="WSC0001">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="WSC0002" package="WSC0002">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="WSC01/2" package="WSC01/2">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="WSC2515" package="WSC2515">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="WSC4527" package="WSC4527">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="WSC6927" package="WSC6927">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R1218" package="R1218">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="1812X7R" package="1812X7R">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="01005" package="R01005">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="cap-master">
<packages>
<package name="0402">
<description>&lt;b&gt;SMD CHIP CAP&lt;/b&gt;&lt;p&gt;
0402, grid 0.0125 inch</description>
<wire x1="-0.245" y1="0.224" x2="0.245" y2="0.224" width="0.1016" layer="51"/>
<wire x1="0.245" y1="-0.224" x2="-0.245" y2="-0.224" width="0.1016" layer="51"/>
<smd name="1" x="-0.635" y="0" dx="0.635" dy="0.9398" layer="1"/>
<smd name="2" x="0.635" y="0" dx="0.635" dy="0.9398" layer="1"/>
<text x="-0.635" y="0.635" size="1.016" layer="25" ratio="14">&gt;NAME</text>
<text x="-0.635" y="-1.5875" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-0.554" y1="-0.3042" x2="-0.254" y2="0.2957" layer="51"/>
<rectangle x1="0.2588" y1="-0.3042" x2="0.5588" y2="0.2957" layer="51"/>
<rectangle x1="-0.1588" y1="-0.3175" x2="0.1588" y2="0.3175" layer="35"/>
</package>
<package name="0603">
<description>&lt;b&gt;SMD CHIP CAP&lt;/b&gt;&lt;p&gt;
0603, grid 0.0125 inch</description>
<wire x1="-0.356" y1="0.432" x2="0.356" y2="0.432" width="0.1016" layer="51"/>
<wire x1="-0.356" y1="-0.419" x2="0.356" y2="-0.419" width="0.1016" layer="51"/>
<smd name="1" x="-0.9525" y="0" dx="1.016" dy="1.016" layer="1"/>
<smd name="2" x="0.9525" y="0" dx="1.016" dy="1.016" layer="1"/>
<text x="-0.9525" y="0.7938" size="1.016" layer="25" ratio="14">&gt;NAME</text>
<text x="-0.9525" y="-1.7463" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-0.8376" y1="-0.4699" x2="-0.3375" y2="0.4801" layer="51"/>
<rectangle x1="0.3302" y1="-0.4699" x2="0.8303" y2="0.4801" layer="51"/>
<rectangle x1="-0.1588" y1="-0.4763" x2="0.1588" y2="0.4763" layer="35"/>
</package>
<package name="0805">
<description>&lt;b&gt;SMD CHIP CAP&lt;/b&gt;&lt;p&gt;
0805, grid 0.0125 inch</description>
<wire x1="-0.381" y1="0.66" x2="0.381" y2="0.66" width="0.1016" layer="51"/>
<wire x1="-0.356" y1="-0.66" x2="0.381" y2="-0.66" width="0.1016" layer="51"/>
<smd name="1" x="-0.9525" y="0" dx="1.27" dy="1.524" layer="1"/>
<smd name="2" x="0.9525" y="0" dx="1.27" dy="1.524" layer="1"/>
<text x="-0.9525" y="0.9525" size="1.016" layer="25" ratio="14">&gt;NAME</text>
<text x="-0.9525" y="-1.905" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-1.0916" y1="-0.7239" x2="-0.3415" y2="0.7262" layer="51"/>
<rectangle x1="0.3556" y1="-0.7239" x2="1.1057" y2="0.7262" layer="51"/>
<rectangle x1="-0.1588" y1="-0.4763" x2="0.1588" y2="0.4763" layer="35"/>
</package>
<package name="1206">
<description>&lt;b&gt;SMD CHIP CAP&lt;/b&gt;&lt;p&gt;
1206, grid 0.0125 inch</description>
<wire x1="-0.965" y1="0.787" x2="0.965" y2="0.787" width="0.1016" layer="51"/>
<wire x1="-0.965" y1="-0.787" x2="0.965" y2="-0.787" width="0.1016" layer="51"/>
<smd name="P$1" x="-1.5875" y="0" dx="1.524" dy="1.778" layer="1"/>
<smd name="P$2" x="1.5875" y="0" dx="1.524" dy="1.778" layer="1"/>
<text x="-1.5875" y="1.1113" size="1.016" layer="25" ratio="14">&gt;NAME</text>
<text x="-1.5875" y="-2.0638" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-1.7012" y1="-0.8509" x2="-0.9511" y2="0.8491" layer="51"/>
<rectangle x1="0.9517" y1="-0.8491" x2="1.7018" y2="0.8509" layer="51"/>
<rectangle x1="-0.3175" y1="-0.7" x2="0.3175" y2="0.7" layer="35"/>
</package>
<package name="1210">
<description>&lt;b&gt;SMD CHIP CAP&lt;/b&gt;&lt;p&gt;
1210, grid 0.0125 inch</description>
<wire x1="-0.9652" y1="1.2446" x2="0.9652" y2="1.2446" width="0.1016" layer="51"/>
<wire x1="-0.9652" y1="-1.2446" x2="0.9652" y2="-1.2446" width="0.1016" layer="51"/>
<smd name="P$1" x="-1.5875" y="0" dx="1.524" dy="2.54" layer="1"/>
<smd name="P$2" x="1.5875" y="0" dx="1.524" dy="2.54" layer="1"/>
<text x="-1.5875" y="1.5875" size="1.016" layer="25" ratio="14">&gt;NAME</text>
<text x="-1.5875" y="-2.54" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-1.7012" y1="-1.2954" x2="-0.9511" y2="1.3045" layer="51"/>
<rectangle x1="0.9517" y1="-1.3039" x2="1.7018" y2="1.296" layer="51"/>
<rectangle x1="-0.3175" y1="-0.7" x2="0.3175" y2="0.7" layer="35"/>
</package>
<package name="1812">
<description>&lt;b&gt;SMD CHIP CAP&lt;/b&gt;&lt;p&gt;
1812, grid 0.0125 inch</description>
<wire x1="-1.4732" y1="1.6002" x2="1.4732" y2="1.6002" width="0.1016" layer="51"/>
<wire x1="-1.4478" y1="-1.6002" x2="1.4732" y2="-1.6002" width="0.1016" layer="51"/>
<smd name="P$1" x="-1.905" y="0" dx="1.905" dy="3.4036" layer="1"/>
<smd name="P$2" x="1.905" y="0" dx="1.905" dy="3.4036" layer="1"/>
<text x="-1.905" y="2.0638" size="1.016" layer="25" ratio="14">&gt;NAME</text>
<text x="-1.905" y="-3.0163" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-2.3876" y1="-1.651" x2="-1.4376" y2="1.649" layer="51"/>
<rectangle x1="1.4478" y1="-1.651" x2="2.3978" y2="1.649" layer="51"/>
<rectangle x1="-0.3175" y1="-0.7" x2="0.3175" y2="0.7" layer="35"/>
</package>
<package name="1812K">
<description>&lt;b&gt;SMD CHIP CAP&lt;/b&gt; - KEMET 1812 Reflow solder&lt;p&gt;
Metric Code Size 4532</description>
<wire x1="-2.175" y1="1.525" x2="2.175" y2="1.525" width="0.1016" layer="51"/>
<wire x1="2.175" y1="-1.525" x2="-2.175" y2="-1.525" width="0.1016" layer="51"/>
<smd name="1" x="-2.2225" y="0" dx="1.8" dy="3.7" layer="1"/>
<smd name="2" x="2.2225" y="0" dx="1.8" dy="3.7" layer="1"/>
<text x="-2.8575" y="2.2225" size="1.016" layer="25" ratio="14">&gt;NAME</text>
<text x="-2.8575" y="-3.175" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-2.25" y1="-1.6" x2="-1.65" y2="1.6" layer="51"/>
<rectangle x1="1.65" y1="-1.6" x2="2.25" y2="1.6" layer="51"/>
<rectangle x1="-0.3175" y1="-0.7" x2="0.3175" y2="0.7" layer="35"/>
</package>
<package name="1825">
<description>&lt;b&gt;SMD CHIP CAP&lt;/b&gt;&lt;p&gt;
chip</description>
<wire x1="-1.4986" y1="3.2766" x2="1.4732" y2="3.2766" width="0.1016" layer="51"/>
<wire x1="-1.4732" y1="-3.2766" x2="1.4986" y2="-3.2766" width="0.1016" layer="51"/>
<smd name="1" x="-1.905" y="0" dx="1.9" dy="6.8" layer="1"/>
<smd name="2" x="1.905" y="0" dx="1.9" dy="6.8" layer="1"/>
<text x="-2.8575" y="3.81" size="1.016" layer="25" ratio="14">&gt;NAME</text>
<text x="-2.8575" y="-4.7625" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-2.413" y1="-3.3528" x2="-1.463" y2="3.3472" layer="51"/>
<rectangle x1="1.4478" y1="-3.3528" x2="2.3978" y2="3.3472" layer="51"/>
<rectangle x1="-0.7" y1="-0.7" x2="0.7" y2="0.7" layer="35"/>
</package>
<package name="1825K">
<description>&lt;b&gt;SMD CHIP CAP&lt;/b&gt; - KEMET 1825 Reflow solder&lt;p&gt;
Metric Code Size 4564</description>
<wire x1="-1.525" y1="3.125" x2="1.525" y2="3.125" width="0.1016" layer="51"/>
<wire x1="1.525" y1="-3.125" x2="-1.525" y2="-3.125" width="0.1016" layer="51"/>
<smd name="1" x="-1.5875" y="0" dx="1.8" dy="6.9" layer="1"/>
<smd name="2" x="1.5875" y="0" dx="1.8" dy="6.9" layer="1"/>
<text x="-2.2225" y="3.81" size="1.016" layer="25" ratio="14">&gt;NAME</text>
<text x="-2.2225" y="-4.7625" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-1.6" y1="-3.2" x2="-1.1" y2="3.2" layer="51"/>
<rectangle x1="1.1" y1="-3.2" x2="1.6" y2="3.2" layer="51"/>
<rectangle x1="-0.3175" y1="-0.635" x2="0.3175" y2="0.635" layer="35"/>
</package>
<package name="2012">
<description>&lt;b&gt;SMD CHIP CAP&lt;/b&gt;&lt;p&gt;
chip</description>
<wire x1="-0.381" y1="0.66" x2="0.381" y2="0.66" width="0.1016" layer="51"/>
<wire x1="-0.356" y1="-0.66" x2="0.381" y2="-0.66" width="0.1016" layer="51"/>
<smd name="1" x="-0.9525" y="0" dx="1.3" dy="1.5" layer="1"/>
<smd name="2" x="0.9525" y="0" dx="1.3" dy="1.5" layer="1"/>
<text x="-1.27" y="0.9525" size="1.016" layer="25" ratio="14">&gt;NAME</text>
<text x="-1.27" y="-1.905" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-1.0917" y1="-0.7239" x2="-0.3416" y2="0.7262" layer="51"/>
<rectangle x1="0.3556" y1="-0.7239" x2="1.1057" y2="0.7262" layer="51"/>
<rectangle x1="-0.1001" y1="-0.4001" x2="0.1001" y2="0.4001" layer="35"/>
</package>
<package name="2220K">
<description>&lt;b&gt;SMD CHIP CAP&lt;/b&gt; - KEMET 2220 Reflow solder&lt;p&gt;
Metric Code Size 5650</description>
<wire x1="-2.725" y1="2.425" x2="2.725" y2="2.425" width="0.1016" layer="51"/>
<wire x1="2.725" y1="-2.425" x2="-2.725" y2="-2.425" width="0.1016" layer="51"/>
<smd name="1" x="-2.54" y="0" dx="1.85" dy="5.5" layer="1"/>
<smd name="2" x="2.8575" y="0" dx="1.85" dy="5.5" layer="1"/>
<text x="-3.4925" y="3.175" size="1.016" layer="25" ratio="14">&gt;NAME</text>
<text x="-3.4925" y="-4.1275" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-2.8" y1="-2.5" x2="-2.2" y2="2.5" layer="51"/>
<rectangle x1="2.2" y1="-2.5" x2="2.8" y2="2.5" layer="51"/>
<rectangle x1="-0.7" y1="-0.7" x2="0.7" y2="0.7" layer="35"/>
</package>
<package name="2225K">
<description>&lt;b&gt;SMD CHIP CAP&lt;/b&gt; - KEMET 2225 Reflow solder&lt;p&gt;
Metric Code Size 5664</description>
<wire x1="-2.725" y1="3.075" x2="2.725" y2="3.075" width="0.1016" layer="51"/>
<wire x1="2.725" y1="-3.075" x2="-2.725" y2="-3.075" width="0.1016" layer="51"/>
<smd name="1" x="-2.54" y="0" dx="1.85" dy="6.8" layer="1"/>
<smd name="2" x="2.54" y="0" dx="1.85" dy="6.8" layer="1"/>
<text x="-3.4925" y="3.81" size="1.016" layer="25" ratio="14">&gt;NAME</text>
<text x="-3.4925" y="-4.7625" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-2.8" y1="-3.15" x2="-2.2" y2="3.15" layer="51"/>
<rectangle x1="2.2" y1="-3.15" x2="2.8" y2="3.15" layer="51"/>
<rectangle x1="-0.7" y1="-0.7" x2="0.7" y2="0.7" layer="35"/>
</package>
<package name="3216">
<description>&lt;b&gt;SMD CHIP CAP&lt;/b&gt;&lt;p&gt;
chip</description>
<wire x1="-0.965" y1="0.787" x2="0.965" y2="0.787" width="0.1016" layer="51"/>
<wire x1="-0.965" y1="-0.787" x2="0.965" y2="-0.787" width="0.1016" layer="51"/>
<smd name="1" x="-1.5875" y="0" dx="1.6" dy="1.8" layer="1"/>
<smd name="2" x="1.5875" y="0" dx="1.6" dy="1.8" layer="1"/>
<text x="-2.2225" y="1.27" size="1.016" layer="25" ratio="14">&gt;NAME</text>
<text x="-2.2225" y="-2.2225" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-1.7013" y1="-0.8509" x2="-0.9512" y2="0.8491" layer="51"/>
<rectangle x1="0.9517" y1="-0.8491" x2="1.7018" y2="0.8509" layer="51"/>
<rectangle x1="-0.3" y1="-0.5001" x2="0.3" y2="0.5001" layer="35"/>
</package>
<package name="3225">
<description>&lt;b&gt;SMD CHIP CAP&lt;/b&gt;&lt;p&gt;
chip</description>
<wire x1="-0.9652" y1="1.2446" x2="0.9652" y2="1.2446" width="0.1016" layer="51"/>
<wire x1="-0.9652" y1="-1.2446" x2="0.9652" y2="-1.2446" width="0.1016" layer="51"/>
<smd name="1" x="-1.4" y="0" dx="1.6" dy="2.7" layer="1"/>
<smd name="2" x="1.4" y="0" dx="1.6" dy="2.7" layer="1"/>
<text x="-2.2225" y="1.5875" size="1.016" layer="25" ratio="14">&gt;NAME</text>
<text x="-2.2225" y="-2.54" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-1.7013" y1="-1.2954" x2="-0.9512" y2="1.3045" layer="51"/>
<rectangle x1="0.9517" y1="-1.304" x2="1.7018" y2="1.2959" layer="51"/>
<rectangle x1="-0.1999" y1="-0.5001" x2="0.1999" y2="0.5001" layer="35"/>
</package>
<package name="4532">
<description>&lt;b&gt;SMD CHIP CAP&lt;/b&gt;&lt;p&gt;
chip</description>
<wire x1="-1.4732" y1="1.6002" x2="1.4732" y2="1.6002" width="0.1016" layer="51"/>
<wire x1="-1.4478" y1="-1.6002" x2="1.4732" y2="-1.6002" width="0.1016" layer="51"/>
<smd name="1" x="-1.905" y="0" dx="1.9" dy="3.4" layer="1"/>
<smd name="2" x="1.905" y="0" dx="1.9" dy="3.4" layer="1"/>
<text x="-2.8575" y="1.905" size="1.016" layer="25" ratio="14">&gt;NAME</text>
<text x="-2.8575" y="-2.8575" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-2.3876" y1="-1.651" x2="-1.4376" y2="1.649" layer="51"/>
<rectangle x1="1.4478" y1="-1.651" x2="2.3978" y2="1.649" layer="51"/>
<rectangle x1="-0.4001" y1="-0.7" x2="0.4001" y2="0.7" layer="35"/>
</package>
<package name="4564">
<description>&lt;b&gt;SMD CHIP CAP&lt;/b&gt;&lt;p&gt;
chip</description>
<wire x1="-1.4986" y1="3.2766" x2="1.4732" y2="3.2766" width="0.1016" layer="51"/>
<wire x1="-1.4732" y1="-3.2766" x2="1.4986" y2="-3.2766" width="0.1016" layer="51"/>
<smd name="1" x="-1.905" y="0" dx="1.9" dy="6.8" layer="1"/>
<smd name="2" x="1.905" y="0" dx="1.9" dy="6.8" layer="1"/>
<text x="-2.8575" y="3.81" size="1.016" layer="25" ratio="14">&gt;NAME</text>
<text x="-2.8575" y="-4.7625" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-2.413" y1="-3.3528" x2="-1.463" y2="3.3472" layer="51"/>
<rectangle x1="1.4478" y1="-3.3528" x2="2.3978" y2="3.3472" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
</package>
<package name="C100-140X060">
<description>&lt;b&gt;NP FILM CAP&lt;/b&gt;&lt;p&gt;
10 mm lead spacing&lt;br&gt; outline 14 x 6 mm&lt;br&gt; grid 0.0125 inch</description>
<wire x1="-0.635" y1="0.9525" x2="-0.635" y2="0" width="0.254" layer="51"/>
<wire x1="-0.635" y1="0" x2="-0.635" y2="-0.9525" width="0.254" layer="51"/>
<wire x1="0.635" y1="0.9525" x2="0.635" y2="0" width="0.254" layer="51"/>
<wire x1="0.635" y1="0" x2="0.635" y2="-0.9525" width="0.254" layer="51"/>
<wire x1="-0.635" y1="0" x2="-5.08" y2="0" width="0.2032" layer="51"/>
<wire x1="0.635" y1="0" x2="5.08" y2="0" width="0.2032" layer="51"/>
<wire x1="7" y1="2" x2="6" y2="3" width="0.2032" layer="21" curve="90"/>
<wire x1="6" y1="3" x2="-6" y2="3" width="0.2032" layer="21"/>
<wire x1="-6" y1="3" x2="-7" y2="2" width="0.2032" layer="21" curve="90"/>
<wire x1="-7" y1="2" x2="-7" y2="-2" width="0.2032" layer="21"/>
<wire x1="-7" y1="-2" x2="-6" y2="-3" width="0.2032" layer="21" curve="90"/>
<wire x1="-6" y1="-3" x2="6" y2="-3" width="0.2032" layer="21"/>
<wire x1="6" y1="-3" x2="7" y2="-2" width="0.2032" layer="21" curve="90"/>
<wire x1="7" y1="-2" x2="7" y2="2" width="0.2032" layer="21"/>
<pad name="1" x="-5.08" y="0" drill="1.016" diameter="2.1844" shape="octagon"/>
<pad name="2" x="5.08" y="0" drill="1.016" diameter="2.1844" shape="octagon"/>
<text x="7.62" y="-1.905" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
<text x="7.62" y="0" size="1.27" layer="25" ratio="14">&gt;NAME</text>
</package>
<package name="C120-150X060">
<description>&lt;b&gt;NP FILM CAP&lt;/b&gt;&lt;p&gt;
12 mm lead spacing&lt;br&gt; outline 15 x 6 mm&lt;br&gt; grid 0.0125 inch</description>
<wire x1="-0.635" y1="1.27" x2="-0.635" y2="0" width="0.254" layer="51"/>
<wire x1="-0.635" y1="0" x2="-0.635" y2="-1.27" width="0.254" layer="51"/>
<wire x1="0.635" y1="1.27" x2="0.635" y2="0" width="0.254" layer="51"/>
<wire x1="0.635" y1="0" x2="0.635" y2="-1.27" width="0.254" layer="51"/>
<wire x1="0.635" y1="0" x2="6.0325" y2="0" width="0.2032" layer="51"/>
<wire x1="-0.635" y1="0" x2="-6.0325" y2="0" width="0.2032" layer="51"/>
<wire x1="-6.5" y1="3" x2="-7.5" y2="2" width="0.2032" layer="21" curve="90"/>
<wire x1="-7.5" y1="2" x2="-7.5" y2="-2" width="0.2032" layer="21"/>
<wire x1="-7.5" y1="-2" x2="-6.5" y2="-3" width="0.2032" layer="21" curve="90"/>
<wire x1="-6.5" y1="-3" x2="6.5" y2="-3" width="0.2032" layer="21"/>
<wire x1="6.5" y1="-3" x2="7.5" y2="-2" width="0.2032" layer="21" curve="90"/>
<wire x1="7.5" y1="-2" x2="7.5" y2="2" width="0.2032" layer="21"/>
<wire x1="7.5" y1="2" x2="6.5" y2="3" width="0.2032" layer="21" curve="90"/>
<wire x1="6.5" y1="3" x2="-6.5" y2="3" width="0.2032" layer="21"/>
<pad name="1" x="-6.0325" y="0" drill="1.016" diameter="2.1844" shape="octagon"/>
<pad name="2" x="6.0325" y="0" drill="1.016" diameter="2.1844" shape="octagon"/>
<text x="8.255" y="-1.905" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
<text x="8.255" y="0" size="1.27" layer="25" ratio="14">&gt;NAME</text>
</package>
<package name="C145-180X070">
<description>&lt;b&gt;NP FILM CAP&lt;/b&gt;&lt;p&gt;
14.5 mm lead spacing&lt;br&gt; outline 18 x 7 mm&lt;br&gt; grid 0.0125 inch</description>
<wire x1="-0.635" y1="1.27" x2="-0.635" y2="0" width="0.254" layer="51"/>
<wire x1="-0.635" y1="0" x2="-0.635" y2="-1.27" width="0.254" layer="51"/>
<wire x1="0.635" y1="1.27" x2="0.635" y2="0" width="0.254" layer="51"/>
<wire x1="0.635" y1="0" x2="0.635" y2="-1.27" width="0.254" layer="51"/>
<wire x1="-0.635" y1="0" x2="-7.3025" y2="0" width="0.2032" layer="51"/>
<wire x1="0.635" y1="0" x2="7.3025" y2="0" width="0.2032" layer="51"/>
<wire x1="-8" y1="3.5" x2="-9" y2="2.5" width="0.2032" layer="21" curve="90"/>
<wire x1="-9" y1="2.5" x2="-9" y2="-2.5" width="0.2032" layer="21"/>
<wire x1="-9" y1="-2.5" x2="-8" y2="-3.5" width="0.2032" layer="21" curve="90"/>
<wire x1="-8" y1="-3.5" x2="8" y2="-3.5" width="0.2032" layer="21"/>
<wire x1="8" y1="-3.5" x2="9" y2="-2.5" width="0.2032" layer="21" curve="90"/>
<wire x1="9" y1="-2.5" x2="9" y2="2.5" width="0.2032" layer="21"/>
<wire x1="9" y1="2.5" x2="8" y2="3.5" width="0.2032" layer="21" curve="90"/>
<wire x1="8" y1="3.5" x2="-8" y2="3.5" width="0.2032" layer="21"/>
<pad name="1" x="-7.3025" y="0" drill="1.016" diameter="2.1844" shape="octagon"/>
<pad name="2" x="7.3025" y="0" drill="1.016" diameter="2.1844" shape="octagon"/>
<text x="9.8425" y="0" size="1.27" layer="25" ratio="14">&gt;NAME</text>
<text x="9.8425" y="-1.905" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C150-190X070">
<description>&lt;b&gt;NP FILM CAP&lt;/b&gt;&lt;p&gt;
15 mm lead spacing&lt;br&gt; outline 19 x 7 mm&lt;br&gt; grid 0.0125 inch</description>
<wire x1="-0.635" y1="1.27" x2="-0.635" y2="0" width="0.254" layer="51"/>
<wire x1="-0.635" y1="0" x2="-0.635" y2="-1.27" width="0.254" layer="51"/>
<wire x1="0.635" y1="1.27" x2="0.635" y2="0" width="0.254" layer="51"/>
<wire x1="0.635" y1="0" x2="0.635" y2="-1.27" width="0.254" layer="51"/>
<wire x1="-0.635" y1="0" x2="-7.62" y2="0" width="0.2032" layer="51"/>
<wire x1="0.635" y1="0" x2="7.62" y2="0" width="0.2032" layer="51"/>
<wire x1="-8.5" y1="3.5" x2="-9.5" y2="2.5" width="0.2032" layer="21" curve="90"/>
<wire x1="-9.5" y1="2.5" x2="-9.5" y2="-2.5" width="0.2032" layer="21"/>
<wire x1="-9.5" y1="-2.5" x2="-8.5" y2="-3.5" width="0.2032" layer="21" curve="90"/>
<wire x1="-8.5" y1="-3.5" x2="8.5" y2="-3.5" width="0.2032" layer="21"/>
<wire x1="8.5" y1="-3.5" x2="9.5" y2="-2.5" width="0.2032" layer="21" curve="90"/>
<wire x1="9.5" y1="-2.5" x2="9.5" y2="2.5" width="0.2032" layer="21"/>
<wire x1="9.5" y1="2.5" x2="8.5" y2="3.5" width="0.2032" layer="21" curve="90"/>
<wire x1="8.5" y1="3.5" x2="-8.5" y2="3.5" width="0.2032" layer="21"/>
<pad name="1" x="-7.62" y="0" drill="1.016" diameter="2.1844" shape="octagon"/>
<pad name="2" x="7.62" y="0" drill="1.016" diameter="2.1844" shape="octagon"/>
<text x="10.16" y="0" size="1.27" layer="25" ratio="14">&gt;NAME</text>
<text x="10.16" y="-1.905" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C170-210X070">
<description>&lt;b&gt;NP FILM CAP&lt;/b&gt;&lt;p&gt;
17 mm lead spacing&lt;br&gt; outline 21 x 7 mm &lt;br&gt; grid 0.0125 inch</description>
<wire x1="-0.635" y1="1.27" x2="-0.635" y2="0" width="0.254" layer="51"/>
<wire x1="-0.635" y1="0" x2="-0.635" y2="-1.27" width="0.254" layer="51"/>
<wire x1="0.635" y1="1.27" x2="0.635" y2="0" width="0.254" layer="51"/>
<wire x1="0.635" y1="0" x2="0.635" y2="-1.27" width="0.254" layer="51"/>
<wire x1="0.635" y1="0" x2="8.5725" y2="0" width="0.2032" layer="51"/>
<wire x1="-0.635" y1="0" x2="-8.5725" y2="0" width="0.2032" layer="51"/>
<wire x1="-9.5" y1="3.5" x2="-10.5" y2="2.5" width="0.2032" layer="21" curve="90"/>
<wire x1="-10.5" y1="2.5" x2="-10.5" y2="-2.5" width="0.2032" layer="21"/>
<wire x1="-10.5" y1="-2.5" x2="-9.5" y2="-3.5" width="0.2032" layer="21" curve="90"/>
<wire x1="-9.5" y1="-3.5" x2="9.5" y2="-3.5" width="0.2032" layer="21"/>
<wire x1="9.5" y1="-3.5" x2="10.5" y2="-2.5" width="0.2032" layer="21" curve="90"/>
<wire x1="10.5" y1="-2.5" x2="10.5" y2="2.5" width="0.2032" layer="21"/>
<wire x1="10.5" y1="2.5" x2="9.5" y2="3.5" width="0.2032" layer="21" curve="90"/>
<wire x1="9.5" y1="3.5" x2="-9.5" y2="3.5" width="0.2032" layer="21"/>
<pad name="1" x="-8.5725" y="0" drill="1.016" diameter="2.1844" shape="octagon"/>
<pad name="2" x="8.5725" y="0" drill="1.016" diameter="2.1844" shape="octagon"/>
<text x="11.1125" y="0" size="1.27" layer="25" ratio="14">&gt;NAME</text>
<text x="11.1125" y="-1.905" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C200-230X080">
<description>&lt;b&gt;NP FILM CAP&lt;/b&gt;&lt;p&gt;
20 mm lead spacing&lt;br&gt; outline 23 x 8 mm&lt;br&gt; grid 0.0125 inch</description>
<wire x1="-0.635" y1="0.9525" x2="-0.635" y2="0" width="0.254" layer="51"/>
<wire x1="-0.635" y1="0" x2="-0.635" y2="-0.9525" width="0.254" layer="51"/>
<wire x1="0.635" y1="0.9525" x2="0.635" y2="0" width="0.254" layer="51"/>
<wire x1="0.635" y1="0" x2="0.635" y2="-0.9525" width="0.254" layer="51"/>
<wire x1="0.635" y1="0" x2="9.8425" y2="0" width="0.2032" layer="51"/>
<wire x1="-0.635" y1="0" x2="-9.8425" y2="0" width="0.2032" layer="51"/>
<wire x1="11.5" y1="-3" x2="10.5" y2="-4" width="0.2032" layer="21" curve="-90"/>
<wire x1="10.5" y1="-4" x2="-10.5" y2="-4" width="0.2032" layer="21"/>
<wire x1="-10.5" y1="-4" x2="-11.5" y2="-3" width="0.2032" layer="21" curve="-90"/>
<wire x1="-11.5" y1="-3" x2="-11.5" y2="3" width="0.2032" layer="21"/>
<wire x1="-11.5" y1="3" x2="-10.5" y2="4" width="0.2032" layer="21" curve="-90"/>
<wire x1="-10.5" y1="4" x2="10.5" y2="4" width="0.2032" layer="21"/>
<wire x1="10.5" y1="4" x2="11.5" y2="3" width="0.2032" layer="21" curve="-90"/>
<wire x1="11.5" y1="3" x2="11.5" y2="-3" width="0.2032" layer="21"/>
<pad name="1" x="-9.8425" y="0" drill="1.016" diameter="2.1844" shape="octagon"/>
<pad name="2" x="9.8425" y="0" drill="1.016" diameter="2.1844" shape="octagon"/>
<text x="12.065" y="-1.905" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
<text x="12.065" y="0" size="1.27" layer="25" ratio="14">&gt;NAME</text>
</package>
<package name="C225-260X090">
<description>&lt;b&gt;NP FILM CAP&lt;/b&gt;&lt;p&gt;
22.5 mm lead spacing&lt;br&gt; outline 26 x 9 mm&lt;br&gt; grid 0.0125 inch</description>
<wire x1="-0.635" y1="1.5875" x2="-0.635" y2="0" width="0.254" layer="51"/>
<wire x1="-0.635" y1="0" x2="-0.635" y2="-1.5875" width="0.254" layer="51"/>
<wire x1="0.635" y1="1.5875" x2="0.635" y2="0" width="0.254" layer="51"/>
<wire x1="0.635" y1="0" x2="0.635" y2="-1.5875" width="0.254" layer="51"/>
<wire x1="-0.635" y1="0" x2="-11.1125" y2="0" width="0.2032" layer="51"/>
<wire x1="0.635" y1="0" x2="11.1125" y2="0" width="0.2032" layer="51"/>
<wire x1="12" y1="-4.5" x2="13" y2="-3.5" width="0.2032" layer="21" curve="90"/>
<wire x1="13" y1="-3.5" x2="13" y2="3.5" width="0.2032" layer="21"/>
<wire x1="13" y1="3.5" x2="12" y2="4.5" width="0.2032" layer="21" curve="90"/>
<wire x1="12" y1="4.5" x2="-12" y2="4.5" width="0.2032" layer="21"/>
<wire x1="-12" y1="4.5" x2="-13" y2="3.5" width="0.2032" layer="21" curve="90"/>
<wire x1="-13" y1="3.5" x2="-13" y2="-3.5" width="0.2032" layer="21"/>
<wire x1="-13" y1="-3.5" x2="-12" y2="-4.5" width="0.2032" layer="21" curve="90"/>
<wire x1="-12" y1="-4.5" x2="12" y2="-4.5" width="0.2032" layer="21"/>
<pad name="1" x="-11.1125" y="0" drill="1.016" diameter="2.1844" shape="octagon"/>
<pad name="2" x="11.1125" y="0" drill="1.016" diameter="2.1844" shape="octagon"/>
<text x="13.6525" y="0" size="1.27" layer="25" ratio="14">&gt;NAME</text>
<text x="13.6525" y="-1.905" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C275-320X100">
<description>&lt;b&gt;NP FILM CAP&lt;/b&gt;&lt;p&gt;
27.5 mm lead spacing&lt;br&gt; outline 32 x 10 mm&lt;br&gt; grid 0.0125 inch</description>
<wire x1="-0.635" y1="1.27" x2="-0.635" y2="0" width="0.254" layer="51"/>
<wire x1="-0.635" y1="0" x2="-0.635" y2="-1.27" width="0.254" layer="51"/>
<wire x1="0.635" y1="1.27" x2="0.635" y2="0" width="0.254" layer="51"/>
<wire x1="0.635" y1="0" x2="0.635" y2="-1.27" width="0.254" layer="51"/>
<wire x1="0.635" y1="0" x2="13.6525" y2="0" width="0.2032" layer="51"/>
<wire x1="-0.635" y1="0" x2="-13.6525" y2="0" width="0.2032" layer="51"/>
<wire x1="-15" y1="-5" x2="-16" y2="-4" width="0.2032" layer="21" curve="-90"/>
<wire x1="-16" y1="-4" x2="-16" y2="4" width="0.2032" layer="21"/>
<wire x1="-16" y1="4" x2="-15" y2="5" width="0.2032" layer="21" curve="-90"/>
<wire x1="-15" y1="5" x2="15" y2="5" width="0.2032" layer="21"/>
<wire x1="15" y1="5" x2="16" y2="4" width="0.2032" layer="21" curve="-90"/>
<wire x1="16" y1="4" x2="16" y2="-4" width="0.2032" layer="21"/>
<wire x1="16" y1="-4" x2="15" y2="-5" width="0.2032" layer="21" curve="-90"/>
<wire x1="15" y1="-5" x2="-15" y2="-5" width="0.2032" layer="21"/>
<pad name="1" x="-13.6525" y="0" drill="1.016" diameter="2.1844" shape="octagon"/>
<pad name="2" x="13.6525" y="0" drill="1.016" diameter="2.1844" shape="octagon"/>
<text x="16.8275" y="0" size="1.27" layer="25" ratio="14">&gt;NAME</text>
<text x="16.8275" y="-1.905" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C035-055X025">
<description>&lt;b&gt;NP FILM CAP&lt;/b&gt;&lt;p&gt; 
3.5 mm lead spacing&lt;br&gt; outline 5.5 x 2.5 mm&lt;br&gt; grid 0.0125 inch</description>
<wire x1="-0.3175" y1="0.635" x2="-0.3175" y2="0" width="0.254" layer="51"/>
<wire x1="-0.3175" y1="0" x2="-0.3175" y2="-0.635" width="0.254" layer="51"/>
<wire x1="0.3175" y1="0.635" x2="0.3175" y2="0" width="0.254" layer="51"/>
<wire x1="0.3175" y1="0" x2="0.3175" y2="-0.635" width="0.254" layer="51"/>
<wire x1="-0.3175" y1="0" x2="-1.5875" y2="0" width="0.2032" layer="51"/>
<wire x1="0.3175" y1="0" x2="1.905" y2="0" width="0.2032" layer="51"/>
<wire x1="-2.0988" y1="1.17" x2="-2.5988" y2="0.67" width="0.2032" layer="21" curve="90"/>
<wire x1="-2.5988" y1="0.67" x2="-2.5988" y2="-0.67" width="0.2032" layer="21"/>
<wire x1="-2.5988" y1="-0.67" x2="-2.0988" y2="-1.17" width="0.2032" layer="21" curve="90"/>
<wire x1="-2.0988" y1="-1.17" x2="2.4163" y2="-1.17" width="0.2032" layer="21"/>
<wire x1="2.4163" y1="-1.17" x2="2.9163" y2="-0.67" width="0.2032" layer="21" curve="90"/>
<wire x1="2.9163" y1="-0.67" x2="2.9163" y2="0.67" width="0.2032" layer="21"/>
<wire x1="2.9163" y1="0.67" x2="2.4163" y2="1.17" width="0.2032" layer="21" curve="90"/>
<wire x1="2.4163" y1="1.17" x2="-2.0988" y2="1.17" width="0.2032" layer="21"/>
<pad name="1" x="-1.5875" y="0" drill="0.8128" diameter="1.6764" shape="octagon"/>
<pad name="2" x="1.905" y="0" drill="0.8128" diameter="1.6764" shape="octagon"/>
<text x="3.81" y="-1.5875" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
<text x="3.81" y="0" size="1.27" layer="25" ratio="14">&gt;NAME</text>
</package>
<package name="C050-075X030">
<description>&lt;b&gt;NP FILM CAP&lt;/b&gt;&lt;p&gt;
5 mm lead spacing&lt;br&gt; outline 7.5 x 3.0 mm&lt;br&gt; grid 0.0125 inch</description>
<wire x1="-0.3175" y1="0.635" x2="-0.3175" y2="0" width="0.254" layer="51"/>
<wire x1="-0.3175" y1="0" x2="-0.3175" y2="-0.635" width="0.254" layer="51"/>
<wire x1="0.3175" y1="0.635" x2="0.3175" y2="0" width="0.254" layer="51"/>
<wire x1="0.3175" y1="0" x2="0.3175" y2="-0.635" width="0.254" layer="51"/>
<wire x1="-0.3175" y1="0" x2="-2.54" y2="0" width="0.2032" layer="51"/>
<wire x1="0.3175" y1="0" x2="2.54" y2="0" width="0.2032" layer="51"/>
<wire x1="-3.75" y1="1" x2="-3.25" y2="1.5" width="0.2032" layer="21" curve="-90"/>
<wire x1="-3.25" y1="1.5" x2="3.25" y2="1.5" width="0.2032" layer="21"/>
<wire x1="3.25" y1="1.5" x2="3.75" y2="1" width="0.2032" layer="21" curve="-90"/>
<wire x1="3.75" y1="1" x2="3.75" y2="-1" width="0.2032" layer="21"/>
<wire x1="3.75" y1="-1" x2="3.25" y2="-1.5" width="0.2032" layer="21" curve="-90"/>
<wire x1="3.25" y1="-1.5" x2="-3.25" y2="-1.5" width="0.2032" layer="21"/>
<wire x1="-3.25" y1="-1.5" x2="-3.75" y2="-1" width="0.2032" layer="21" curve="-90"/>
<wire x1="-3.75" y1="-1" x2="-3.75" y2="1" width="0.2032" layer="21"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" diameter="1.6764" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" diameter="1.6764" shape="octagon"/>
<text x="4.445" y="0" size="1.27" layer="25" ratio="14">&gt;NAME</text>
<text x="4.445" y="-1.5875" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C075-110X050">
<description>&lt;b&gt;NP FILM CAP&lt;/b&gt;&lt;p&gt;
7.5 mm lead spacing&lt;br&gt; outline 11 x 5 mm&lt;br&gt; grid 0.0125 inch</description>
<wire x1="-0.3175" y1="0.9525" x2="-0.3175" y2="0" width="0.254" layer="51"/>
<wire x1="-0.3175" y1="0" x2="-0.3175" y2="-0.9525" width="0.254" layer="51"/>
<wire x1="0.3175" y1="0.9525" x2="0.3175" y2="0" width="0.254" layer="51"/>
<wire x1="0.3175" y1="0" x2="0.3175" y2="-0.9525" width="0.254" layer="51"/>
<wire x1="-0.3175" y1="0" x2="-3.81" y2="0" width="0.2032" layer="51"/>
<wire x1="0.3175" y1="0" x2="3.81" y2="0" width="0.2032" layer="51"/>
<wire x1="4.7625" y1="-2.54" x2="5.3975" y2="-1.905" width="0.2032" layer="21" curve="90"/>
<wire x1="5.3975" y1="-1.905" x2="5.3975" y2="1.905" width="0.2032" layer="21"/>
<wire x1="5.3975" y1="1.905" x2="4.7625" y2="2.54" width="0.2032" layer="21" curve="90"/>
<wire x1="4.7625" y1="2.54" x2="-4.7625" y2="2.54" width="0.2032" layer="21"/>
<wire x1="-4.7625" y1="2.54" x2="-5.3975" y2="1.905" width="0.2032" layer="21" curve="90"/>
<wire x1="-5.3975" y1="1.905" x2="-5.3975" y2="-1.905" width="0.2032" layer="21"/>
<wire x1="-5.3975" y1="-1.905" x2="-4.7625" y2="-2.54" width="0.2032" layer="21" curve="90"/>
<wire x1="-4.7625" y1="-2.54" x2="4.7625" y2="-2.54" width="0.2032" layer="21"/>
<pad name="1" x="-3.81" y="0" drill="0.8128" diameter="1.778" shape="octagon"/>
<pad name="2" x="3.81" y="0" drill="0.8128" diameter="1.778" shape="octagon"/>
<text x="6.0325" y="-1.905" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
<text x="6.0325" y="0" size="1.27" layer="25" ratio="14">&gt;NAME</text>
</package>
<package name="C085-110X050">
<description>&lt;b&gt;NP FILM CAP&lt;/b&gt;&lt;p&gt;
8.5 mm lead spacing&lt;br&gt; outline 11 x 5 mm&lt;br&gt; grid 0.0125 inch</description>
<wire x1="-0.3175" y1="0.9525" x2="-0.3175" y2="0" width="0.254" layer="51"/>
<wire x1="-0.3175" y1="0" x2="-0.3175" y2="-0.9525" width="0.254" layer="51"/>
<wire x1="0.3175" y1="0.9525" x2="0.3175" y2="0" width="0.254" layer="51"/>
<wire x1="0.3175" y1="0" x2="0.3175" y2="-0.9525" width="0.254" layer="51"/>
<wire x1="-0.3175" y1="0" x2="-4.1275" y2="0" width="0.2032" layer="51"/>
<wire x1="0.3175" y1="0" x2="4.1275" y2="0" width="0.2032" layer="51"/>
<wire x1="5.5" y1="2" x2="5" y2="2.5" width="0.2032" layer="21" curve="90"/>
<wire x1="5" y1="2.5" x2="-5" y2="2.5" width="0.2032" layer="21"/>
<wire x1="-5" y1="2.5" x2="-5.5" y2="2" width="0.2032" layer="21" curve="90"/>
<wire x1="-5.5" y1="2" x2="-5.5" y2="-2" width="0.2032" layer="21"/>
<wire x1="-5.5" y1="-2" x2="-5" y2="-2.5" width="0.2032" layer="21" curve="90"/>
<wire x1="-5" y1="-2.5" x2="5" y2="-2.5" width="0.2032" layer="21"/>
<wire x1="5" y1="-2.5" x2="5.5" y2="-2" width="0.2032" layer="21" curve="90"/>
<wire x1="5.5" y1="-2" x2="5.5" y2="2" width="0.2032" layer="21"/>
<pad name="1" x="-4.1275" y="0" drill="0.8128" diameter="1.778" shape="octagon"/>
<pad name="2" x="4.1275" y="0" drill="0.8128" diameter="1.778" shape="octagon"/>
<text x="6.0325" y="0" size="1.27" layer="25" ratio="14">&gt;NAME</text>
<text x="6.0325" y="-1.905" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C275-310X150">
<description>&lt;b&gt;NP FILM CAP&lt;/b&gt;&lt;p&gt;
27.5 mm lead spacing&lt;br&gt; outline 31 x 15 mm&lt;br&gt; grid 0.0125 inch</description>
<wire x1="-0.635" y1="1.27" x2="-0.635" y2="0" width="0.254" layer="51"/>
<wire x1="-0.635" y1="0" x2="-0.635" y2="-1.27" width="0.254" layer="51"/>
<wire x1="0.635" y1="1.27" x2="0.635" y2="0" width="0.254" layer="51"/>
<wire x1="0.635" y1="0" x2="0.635" y2="-1.27" width="0.254" layer="51"/>
<wire x1="0.635" y1="0" x2="13.6525" y2="0" width="0.2032" layer="51"/>
<wire x1="-0.635" y1="0" x2="-13.6525" y2="0" width="0.2032" layer="51"/>
<wire x1="14.5" y1="-7.5" x2="15.5" y2="-6.5" width="0.2032" layer="21" curve="90"/>
<wire x1="15.5" y1="-6.5" x2="15.5" y2="6.5" width="0.2032" layer="21"/>
<wire x1="15.5" y1="6.5" x2="14.5" y2="7.5" width="0.2032" layer="21" curve="90"/>
<wire x1="14.5" y1="7.5" x2="-14.5" y2="7.5" width="0.2032" layer="21"/>
<wire x1="-14.5" y1="7.5" x2="-15.5" y2="6.5" width="0.2032" layer="21" curve="90"/>
<wire x1="-15.5" y1="6.5" x2="-15.5" y2="-6.5" width="0.2032" layer="21"/>
<wire x1="-15.5" y1="-6.5" x2="-14.5" y2="-7.5" width="0.2032" layer="21" curve="90"/>
<wire x1="-14.5" y1="-7.5" x2="14.5" y2="-7.5" width="0.2032" layer="21"/>
<pad name="1" x="-13.6525" y="0" drill="1.016" diameter="2.1844" shape="octagon"/>
<pad name="2" x="13.6525" y="0" drill="1.016" diameter="2.1844" shape="octagon"/>
<text x="16.1925" y="0" size="1.27" layer="25" ratio="14">&gt;NAME</text>
<text x="16.1925" y="-1.905" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C050-075X040">
<description>&lt;b&gt;NP FILM CAP&lt;/b&gt;&lt;p&gt;
5 mm lead spacing&lt;br&gt; outline 7.5 x 4.0 mm&lt;br&gt; grid 0.0125 inch</description>
<wire x1="-0.3175" y1="0.635" x2="-0.3175" y2="0" width="0.254" layer="51"/>
<wire x1="-0.3175" y1="0" x2="-0.3175" y2="-0.635" width="0.254" layer="51"/>
<wire x1="0.3175" y1="0.635" x2="0.3175" y2="0" width="0.254" layer="51"/>
<wire x1="0.3175" y1="0" x2="0.3175" y2="-0.635" width="0.254" layer="51"/>
<wire x1="-0.3175" y1="0" x2="-2.54" y2="0" width="0.2032" layer="51"/>
<wire x1="0.3175" y1="0" x2="2.54" y2="0" width="0.2032" layer="51"/>
<wire x1="-3.75" y1="-1.5" x2="-3.25" y2="-2" width="0.2032" layer="21" curve="90"/>
<wire x1="-3.25" y1="-2" x2="3.25" y2="-2" width="0.2032" layer="21"/>
<wire x1="3.25" y1="-2" x2="3.75" y2="-1.5" width="0.2032" layer="21" curve="90"/>
<wire x1="3.75" y1="-1.5" x2="3.75" y2="1.5" width="0.2032" layer="21"/>
<wire x1="3.75" y1="1.5" x2="3.25" y2="2" width="0.2032" layer="21" curve="90"/>
<wire x1="3.25" y1="2" x2="-3.25" y2="2" width="0.2032" layer="21"/>
<wire x1="-3.25" y1="2" x2="-3.75" y2="1.5" width="0.2032" layer="21" curve="90"/>
<wire x1="-3.75" y1="1.5" x2="-3.75" y2="-1.5" width="0.2032" layer="21"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" diameter="1.6764" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" diameter="1.6764" shape="octagon"/>
<text x="4.445" y="0" size="1.27" layer="25" ratio="14">&gt;NAME</text>
<text x="4.445" y="-1.5875" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C060-090X030">
<description>&lt;b&gt;NP FILM CAP&lt;/b&gt;&lt;p&gt;
6 mm lead spacing&lt;br&gt; outline 9.00 x 3.0 mm&lt;br&gt; grid 0.00625 inch</description>
<wire x1="3.8825" y1="-1.5" x2="4.3825" y2="-1" width="0.2032" layer="21" curve="90"/>
<wire x1="4.3825" y1="-1" x2="4.3825" y2="1" width="0.2032" layer="21"/>
<wire x1="4.3825" y1="1" x2="3.8825" y2="1.5" width="0.2032" layer="21" curve="90"/>
<wire x1="3.8825" y1="1.5" x2="-4.1175" y2="1.5" width="0.2032" layer="21"/>
<wire x1="-4.1175" y1="1.5" x2="-4.6175" y2="1" width="0.2032" layer="21" curve="90"/>
<wire x1="-4.6175" y1="1" x2="-4.6175" y2="-1" width="0.2032" layer="21"/>
<wire x1="-4.6175" y1="-1" x2="-4.1175" y2="-1.5" width="0.2032" layer="21" curve="90"/>
<wire x1="-4.1175" y1="-1.5" x2="3.8825" y2="-1.5" width="0.2032" layer="21"/>
<wire x1="-0.3175" y1="0.635" x2="-0.3175" y2="0" width="0.254" layer="51"/>
<wire x1="-0.3175" y1="0" x2="-0.3175" y2="-0.635" width="0.254" layer="51"/>
<wire x1="0.3175" y1="0.635" x2="0.3175" y2="0" width="0.254" layer="51"/>
<wire x1="0.3175" y1="0" x2="0.3175" y2="-0.635" width="0.254" layer="51"/>
<wire x1="-0.3175" y1="0" x2="-2.8575" y2="0" width="0.2032" layer="51"/>
<wire x1="0.3175" y1="0" x2="2.54" y2="0" width="0.2032" layer="51"/>
<pad name="1" x="-3.1749" y="0" drill="0.8128" diameter="1.9304" shape="octagon"/>
<pad name="2" x="2.8576" y="0" drill="0.8128" diameter="1.9304" shape="octagon"/>
<text x="4.7626" y="0" size="1.27" layer="25" ratio="14">&gt;NAME</text>
<text x="4.7626" y="-1.5875" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C050-080X045">
<description>&lt;b&gt;NP FILM CAP&lt;/b&gt;&lt;p&gt;
5 mm lead spacing&lt;br&gt; outline 8.0 x 4.5 mm&lt;br&gt; grid 0.0125 inch</description>
<wire x1="-0.3175" y1="0.635" x2="-0.3175" y2="0" width="0.254" layer="51"/>
<wire x1="-0.3175" y1="0" x2="-0.3175" y2="-0.635" width="0.254" layer="51"/>
<wire x1="0.3175" y1="0.635" x2="0.3175" y2="0" width="0.254" layer="51"/>
<wire x1="0.3175" y1="0" x2="0.3175" y2="-0.635" width="0.254" layer="51"/>
<wire x1="-0.3175" y1="0" x2="-2.54" y2="0" width="0.2032" layer="51"/>
<wire x1="0.3175" y1="0" x2="2.54" y2="0" width="0.2032" layer="51"/>
<wire x1="-4" y1="-1.75" x2="-3.5" y2="-2.25" width="0.2032" layer="21" curve="90"/>
<wire x1="-3.5" y1="-2.25" x2="3.5" y2="-2.25" width="0.2032" layer="21"/>
<wire x1="3.5" y1="-2.25" x2="4" y2="-1.75" width="0.2032" layer="21" curve="90"/>
<wire x1="4" y1="-1.75" x2="4" y2="1.75" width="0.2032" layer="21"/>
<wire x1="4" y1="1.75" x2="3.5" y2="2.25" width="0.2032" layer="21" curve="90"/>
<wire x1="3.5" y1="2.25" x2="-3.5" y2="2.25" width="0.2032" layer="21"/>
<wire x1="-3.5" y1="2.25" x2="-4" y2="1.75" width="0.2032" layer="21" curve="90"/>
<wire x1="-4" y1="1.75" x2="-4" y2="-1.75" width="0.2032" layer="21"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" diameter="1.6764" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" diameter="1.6764" shape="octagon"/>
<text x="4.445" y="0" size="1.27" layer="25" ratio="14">&gt;NAME</text>
<text x="4.445" y="-1.905" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C075-110X060">
<description>&lt;b&gt;NP FILM CAP&lt;/b&gt;&lt;p&gt;
7.5 mm lead spacing&lt;br&gt; outline 11 x 6 mm&lt;br&gt; grid 0.0125 inch</description>
<wire x1="-0.3175" y1="0.9525" x2="-0.3175" y2="0" width="0.254" layer="51"/>
<wire x1="-0.3175" y1="0" x2="-0.3175" y2="-0.9525" width="0.254" layer="51"/>
<wire x1="0.3175" y1="0.9525" x2="0.3175" y2="0" width="0.254" layer="51"/>
<wire x1="0.3175" y1="0" x2="0.3175" y2="-0.9525" width="0.254" layer="51"/>
<wire x1="-0.3175" y1="0" x2="-3.81" y2="0" width="0.2032" layer="51"/>
<wire x1="0.3175" y1="0" x2="3.81" y2="0" width="0.2032" layer="51"/>
<wire x1="4.7625" y1="-2.8575" x2="5.3975" y2="-2.2225" width="0.2032" layer="21" curve="90"/>
<wire x1="5.3975" y1="-2.2225" x2="5.3975" y2="2.2225" width="0.2032" layer="21"/>
<wire x1="5.3975" y1="2.2225" x2="4.7625" y2="2.8575" width="0.2032" layer="21" curve="90"/>
<wire x1="4.7625" y1="2.8575" x2="-4.7625" y2="2.8575" width="0.2032" layer="21"/>
<wire x1="-4.7625" y1="2.8575" x2="-5.3975" y2="2.2225" width="0.2032" layer="21" curve="90"/>
<wire x1="-5.3975" y1="2.2225" x2="-5.3975" y2="-2.2225" width="0.2032" layer="21"/>
<wire x1="-5.3975" y1="-2.2225" x2="-4.7625" y2="-2.8575" width="0.2032" layer="21" curve="90"/>
<wire x1="-4.7625" y1="-2.8575" x2="4.7625" y2="-2.8575" width="0.2032" layer="21"/>
<pad name="1" x="-3.81" y="0" drill="0.8128" diameter="1.778" shape="octagon"/>
<pad name="2" x="3.81" y="0" drill="0.8128" diameter="1.778" shape="octagon"/>
<text x="6.0325" y="-1.905" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
<text x="6.0325" y="0" size="1.27" layer="25" ratio="14">&gt;NAME</text>
</package>
<package name="C075-110X070">
<description>&lt;b&gt;NP FILM CAP&lt;/b&gt;&lt;p&gt;
7.5 mm lead spacing&lt;br&gt; outline 11 x 7 mm&lt;br&gt; grid 0.0125 inch</description>
<wire x1="-0.3175" y1="0.9525" x2="-0.3175" y2="0" width="0.254" layer="51"/>
<wire x1="-0.3175" y1="0" x2="-0.3175" y2="-0.9525" width="0.254" layer="51"/>
<wire x1="0.3175" y1="0.9525" x2="0.3175" y2="0" width="0.254" layer="51"/>
<wire x1="0.3175" y1="0" x2="0.3175" y2="-0.9525" width="0.254" layer="51"/>
<wire x1="-0.3175" y1="0" x2="-3.81" y2="0" width="0.2032" layer="51"/>
<wire x1="0.3175" y1="0" x2="3.81" y2="0" width="0.2032" layer="51"/>
<wire x1="4.7625" y1="-3.175" x2="5.3975" y2="-2.54" width="0.2032" layer="21" curve="90"/>
<wire x1="5.3975" y1="-2.54" x2="5.3975" y2="2.54" width="0.2032" layer="21"/>
<wire x1="5.3975" y1="2.54" x2="4.7625" y2="3.175" width="0.2032" layer="21" curve="90"/>
<wire x1="4.7625" y1="3.175" x2="-4.7625" y2="3.175" width="0.2032" layer="21"/>
<wire x1="-4.7625" y1="3.175" x2="-5.3975" y2="2.54" width="0.2032" layer="21" curve="90"/>
<wire x1="-5.3975" y1="2.54" x2="-5.3975" y2="-2.54" width="0.2032" layer="21"/>
<wire x1="-5.3975" y1="-2.54" x2="-4.7625" y2="-3.175" width="0.2032" layer="21" curve="90"/>
<wire x1="-4.7625" y1="-3.175" x2="4.7625" y2="-3.175" width="0.2032" layer="21"/>
<pad name="1" x="-3.81" y="0" drill="0.8128" diameter="1.778" shape="octagon"/>
<pad name="2" x="3.81" y="0" drill="0.8128" diameter="1.778" shape="octagon"/>
<text x="6.0325" y="-1.905" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
<text x="6.0325" y="0" size="1.27" layer="25" ratio="14">&gt;NAME</text>
</package>
<package name="C050-110X055">
<description>&lt;b&gt;NP FILM CAP&lt;/b&gt;&lt;p&gt;
5 mm lead spacing&lt;br&gt; outline 11.0 x 5.5 mm&lt;br&gt; grid 0.0125 inch</description>
<wire x1="-0.3175" y1="0.635" x2="-0.3175" y2="0" width="0.254" layer="51"/>
<wire x1="-0.3175" y1="0" x2="-0.3175" y2="-0.635" width="0.254" layer="51"/>
<wire x1="0.3175" y1="0.635" x2="0.3175" y2="0" width="0.254" layer="51"/>
<wire x1="0.3175" y1="0" x2="0.3175" y2="-0.635" width="0.254" layer="51"/>
<wire x1="-0.3175" y1="0" x2="-2.54" y2="0" width="0.2032" layer="51"/>
<wire x1="0.3175" y1="0" x2="2.54" y2="0" width="0.2032" layer="51"/>
<wire x1="-5" y1="-2.25" x2="-4.5" y2="-2.75" width="0.2032" layer="21" curve="90"/>
<wire x1="-4.5" y1="-2.75" x2="4.5" y2="-2.75" width="0.2032" layer="21"/>
<wire x1="4.5" y1="-2.75" x2="5" y2="-2.25" width="0.2032" layer="21" curve="90"/>
<wire x1="5" y1="-2.25" x2="5" y2="2.25" width="0.2032" layer="21"/>
<wire x1="5" y1="2.25" x2="4.5" y2="2.75" width="0.2032" layer="21" curve="90"/>
<wire x1="4.5" y1="2.75" x2="-4.5" y2="2.75" width="0.2032" layer="21"/>
<wire x1="-4.5" y1="2.75" x2="-5" y2="2.25" width="0.2032" layer="21" curve="90"/>
<wire x1="-5" y1="2.25" x2="-5" y2="-2.25" width="0.2032" layer="21"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" diameter="1.6764" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" diameter="1.6764" shape="octagon"/>
<text x="5.7625" y="0" size="1.27" layer="25" ratio="14">&gt;NAME</text>
<text x="5.7625" y="-1.905" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C075-105X035">
<description>&lt;b&gt;NP FILM CAP&lt;/b&gt;&lt;p&gt;
7.5 mm lead spacing&lt;br&gt; outline 10.5 x 3.5 mm&lt;br&gt; grid 0.0125 inch</description>
<wire x1="-0.3175" y1="0.9525" x2="-0.3175" y2="0" width="0.254" layer="51"/>
<wire x1="-0.3175" y1="0" x2="-0.3175" y2="-0.9525" width="0.254" layer="51"/>
<wire x1="0.3175" y1="0.9525" x2="0.3175" y2="0" width="0.254" layer="51"/>
<wire x1="0.3175" y1="0" x2="0.3175" y2="-0.9525" width="0.254" layer="51"/>
<wire x1="-0.3175" y1="0" x2="-3.81" y2="0" width="0.2032" layer="51"/>
<wire x1="0.3175" y1="0" x2="3.81" y2="0" width="0.2032" layer="51"/>
<wire x1="4.5125" y1="-1.79" x2="5.1475" y2="-1.155" width="0.2032" layer="21" curve="90"/>
<wire x1="5.1475" y1="-1.155" x2="5.1475" y2="1.155" width="0.2032" layer="21"/>
<wire x1="5.1475" y1="1.155" x2="4.5125" y2="1.79" width="0.2032" layer="21" curve="90"/>
<wire x1="4.5125" y1="1.79" x2="-4.5125" y2="1.79" width="0.2032" layer="21"/>
<wire x1="-4.5125" y1="1.79" x2="-5.1475" y2="1.155" width="0.2032" layer="21" curve="90"/>
<wire x1="-5.1475" y1="1.155" x2="-5.1475" y2="-1.155" width="0.2032" layer="21"/>
<wire x1="-5.1475" y1="-1.155" x2="-4.5125" y2="-1.79" width="0.2032" layer="21" curve="90"/>
<wire x1="-4.5125" y1="-1.79" x2="4.5125" y2="-1.79" width="0.2032" layer="21"/>
<pad name="1" x="-3.81" y="0" drill="0.8128" diameter="1.778" shape="octagon"/>
<pad name="2" x="3.81" y="0" drill="0.8128" diameter="1.778" shape="octagon"/>
<text x="6.0325" y="-1.905" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
<text x="6.0325" y="0" size="1.27" layer="25" ratio="14">&gt;NAME</text>
</package>
<package name="C050-075X035">
<description>&lt;b&gt;NP FILM CAP&lt;/b&gt;&lt;p&gt;
5 mm lead spacing&lt;br&gt; outline 7.5 x 3.5 mm&lt;br&gt; grid 0.0125 inch</description>
<wire x1="-0.3175" y1="0.635" x2="-0.3175" y2="0" width="0.254" layer="51"/>
<wire x1="-0.3175" y1="0" x2="-0.3175" y2="-0.635" width="0.254" layer="51"/>
<wire x1="0.3175" y1="0.635" x2="0.3175" y2="0" width="0.254" layer="51"/>
<wire x1="0.3175" y1="0" x2="0.3175" y2="-0.635" width="0.254" layer="51"/>
<wire x1="-0.3175" y1="0" x2="-2.54" y2="0" width="0.2032" layer="51"/>
<wire x1="0.3175" y1="0" x2="2.54" y2="0" width="0.2032" layer="51"/>
<wire x1="-3.75" y1="-1.25" x2="-3.25" y2="-1.75" width="0.2032" layer="21" curve="90"/>
<wire x1="-3.25" y1="-1.75" x2="3.25" y2="-1.75" width="0.2032" layer="21"/>
<wire x1="3.25" y1="-1.75" x2="3.75" y2="-1.25" width="0.2032" layer="21" curve="90"/>
<wire x1="3.75" y1="-1.25" x2="3.75" y2="1.25" width="0.2032" layer="21"/>
<wire x1="3.75" y1="1.25" x2="3.25" y2="1.75" width="0.2032" layer="21" curve="90"/>
<wire x1="3.25" y1="1.75" x2="-3.25" y2="1.75" width="0.2032" layer="21"/>
<wire x1="-3.25" y1="1.75" x2="-3.75" y2="1.25" width="0.2032" layer="21" curve="90"/>
<wire x1="-3.75" y1="1.25" x2="-3.75" y2="-1.25" width="0.2032" layer="21"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" diameter="1.6764" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" diameter="1.6764" shape="octagon"/>
<text x="4.445" y="0" size="1.27" layer="25" ratio="14">&gt;NAME</text>
<text x="4.445" y="-1.5875" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C050-075X080">
<description>&lt;b&gt;NP FILM CAP&lt;/b&gt;&lt;p&gt;
5 mm lead spacing&lt;br&gt; outline 7.5 x 8.0 mm&lt;br&gt; grid 0.0125 inch</description>
<wire x1="-0.3175" y1="0.635" x2="-0.3175" y2="0" width="0.254" layer="51"/>
<wire x1="-0.3175" y1="0" x2="-0.3175" y2="-0.635" width="0.254" layer="51"/>
<wire x1="0.3175" y1="0.635" x2="0.3175" y2="0" width="0.254" layer="51"/>
<wire x1="0.3175" y1="0" x2="0.3175" y2="-0.635" width="0.254" layer="51"/>
<wire x1="-0.3175" y1="0" x2="-2.54" y2="0" width="0.2032" layer="51"/>
<wire x1="0.3175" y1="0" x2="2.54" y2="0" width="0.2032" layer="51"/>
<wire x1="-3.75" y1="-3.5" x2="-3.25" y2="-4" width="0.2032" layer="21" curve="90"/>
<wire x1="-3.25" y1="-4" x2="3.25" y2="-4" width="0.2032" layer="21"/>
<wire x1="3.25" y1="-4" x2="3.75" y2="-3.5" width="0.2032" layer="21" curve="90"/>
<wire x1="3.75" y1="-3.5" x2="3.75" y2="3.5" width="0.2032" layer="21"/>
<wire x1="3.75" y1="3.5" x2="3.25" y2="4" width="0.2032" layer="21" curve="90"/>
<wire x1="3.25" y1="4" x2="-3.25" y2="4" width="0.2032" layer="21"/>
<wire x1="-3.25" y1="4" x2="-3.75" y2="3.5" width="0.2032" layer="21" curve="90"/>
<wire x1="-3.75" y1="3.5" x2="-3.75" y2="-3.5" width="0.2032" layer="21"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" diameter="1.6764" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" diameter="1.6764" shape="octagon"/>
<text x="4.445" y="0" size="1.27" layer="25" ratio="14">&gt;NAME</text>
<text x="4.445" y="-1.905" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C072-095X030">
<description>&lt;b&gt;NP FILM CAP&lt;/b&gt;&lt;p&gt;
7.2 mm lead spacing&lt;br&gt; outline 9.5 x 3.0 mm&lt;br&gt; grid 0.0125 inch</description>
<wire x1="-0.3175" y1="0.9525" x2="-0.3175" y2="0" width="0.254" layer="51"/>
<wire x1="-0.3175" y1="0" x2="-0.3175" y2="-0.9525" width="0.254" layer="51"/>
<wire x1="0.3175" y1="0.9525" x2="0.3175" y2="0" width="0.254" layer="51"/>
<wire x1="0.3175" y1="0" x2="0.3175" y2="-0.9525" width="0.254" layer="51"/>
<wire x1="-0.3175" y1="0" x2="-3.56" y2="0" width="0.2032" layer="51"/>
<wire x1="0.3175" y1="0" x2="3.56" y2="0" width="0.2032" layer="51"/>
<wire x1="4.195" y1="-1.4725" x2="4.83" y2="-0.8375" width="0.2032" layer="21" curve="90"/>
<wire x1="4.83" y1="-0.8375" x2="4.83" y2="0.8375" width="0.2032" layer="21"/>
<wire x1="4.83" y1="0.8375" x2="4.195" y2="1.4725" width="0.2032" layer="21" curve="90"/>
<wire x1="4.195" y1="1.4725" x2="-4.195" y2="1.4725" width="0.2032" layer="21"/>
<wire x1="-4.195" y1="1.4725" x2="-4.83" y2="0.8375" width="0.2032" layer="21" curve="90"/>
<wire x1="-4.83" y1="0.8375" x2="-4.83" y2="-0.8375" width="0.2032" layer="21"/>
<wire x1="-4.83" y1="-0.8375" x2="-4.195" y2="-1.4725" width="0.2032" layer="21" curve="90"/>
<wire x1="-4.195" y1="-1.4725" x2="4.195" y2="-1.4725" width="0.2032" layer="21"/>
<pad name="1" x="-3.6" y="0" drill="0.8128" diameter="1.778" shape="octagon"/>
<pad name="2" x="3.6" y="0" drill="0.8128" diameter="1.778" shape="octagon"/>
<text x="5.5325" y="-1.405" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
<text x="5.5325" y="0.5" size="1.27" layer="25" ratio="14">&gt;NAME</text>
</package>
<package name="C072-095X035">
<description>&lt;b&gt;NP FILM CAP&lt;/b&gt;&lt;p&gt;
7.2 mm lead spacing&lt;br&gt; outline 9.5 x 3.5 mm&lt;br&gt; grid 0.0125 inch</description>
<wire x1="-0.3175" y1="0.9525" x2="-0.3175" y2="0" width="0.254" layer="51"/>
<wire x1="-0.3175" y1="0" x2="-0.3175" y2="-0.9525" width="0.254" layer="51"/>
<wire x1="0.3175" y1="0.9525" x2="0.3175" y2="0" width="0.254" layer="51"/>
<wire x1="0.3175" y1="0" x2="0.3175" y2="-0.9525" width="0.254" layer="51"/>
<wire x1="-0.3175" y1="0" x2="-3.56" y2="0" width="0.2032" layer="51"/>
<wire x1="0.3175" y1="0" x2="3.56" y2="0" width="0.2032" layer="51"/>
<wire x1="4.195" y1="-1.8138" x2="4.83" y2="-1.1788" width="0.2032" layer="21" curve="90"/>
<wire x1="4.83" y1="-1.1788" x2="4.83" y2="1.1787" width="0.2032" layer="21"/>
<wire x1="4.83" y1="1.1787" x2="4.195" y2="1.8137" width="0.2032" layer="21" curve="90"/>
<wire x1="4.195" y1="1.8137" x2="-4.195" y2="1.8137" width="0.2032" layer="21"/>
<wire x1="-4.195" y1="1.8137" x2="-4.83" y2="1.1787" width="0.2032" layer="21" curve="90"/>
<wire x1="-4.83" y1="1.1787" x2="-4.83" y2="-1.1788" width="0.2032" layer="21"/>
<wire x1="-4.83" y1="-1.1788" x2="-4.195" y2="-1.8138" width="0.2032" layer="21" curve="90"/>
<wire x1="-4.195" y1="-1.8138" x2="4.195" y2="-1.8138" width="0.2032" layer="21"/>
<pad name="1" x="-3.6" y="0" drill="0.8128" diameter="1.778" shape="octagon"/>
<pad name="2" x="3.6" y="0" drill="0.8128" diameter="1.778" shape="octagon"/>
<text x="5.5325" y="-1.405" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
<text x="5.5325" y="0.5" size="1.27" layer="25" ratio="14">&gt;NAME</text>
</package>
<package name="C080-110X045">
<description>&lt;b&gt;NP FILM CAP&lt;/b&gt;&lt;p&gt;
8.0 mm lead spacing&lt;br&gt; outline 11 x 0 mm&lt;br&gt; grid 0.00625 inch</description>
<wire x1="-0.3175" y1="0.9525" x2="-0.3175" y2="0" width="0.254" layer="51"/>
<wire x1="-0.3175" y1="0" x2="-0.3175" y2="-0.9525" width="0.254" layer="51"/>
<wire x1="0.3175" y1="0.9525" x2="0.3175" y2="0" width="0.254" layer="51"/>
<wire x1="0.3175" y1="0" x2="0.3175" y2="-0.9525" width="0.254" layer="51"/>
<wire x1="-0.3175" y1="0" x2="-3.8775" y2="0" width="0.2032" layer="51"/>
<wire x1="0.3175" y1="0" x2="3.8775" y2="0" width="0.2032" layer="51"/>
<wire x1="5.25" y1="1.8413" x2="4.75" y2="2.3413" width="0.2032" layer="21" curve="90"/>
<wire x1="4.75" y1="2.3413" x2="-4.75" y2="2.3413" width="0.2032" layer="21"/>
<wire x1="-4.75" y1="2.3413" x2="-5.25" y2="1.8413" width="0.2032" layer="21" curve="90"/>
<wire x1="-5.25" y1="1.8413" x2="-5.25" y2="-1.8413" width="0.2032" layer="21"/>
<wire x1="-5.25" y1="-1.8413" x2="-4.75" y2="-2.3413" width="0.2032" layer="21" curve="90"/>
<wire x1="-4.75" y1="-2.3413" x2="4.75" y2="-2.3413" width="0.2032" layer="21"/>
<wire x1="4.75" y1="-2.3413" x2="5.25" y2="-1.8413" width="0.2032" layer="21" curve="90"/>
<wire x1="5.25" y1="-1.8413" x2="5.25" y2="1.8413" width="0.2032" layer="21"/>
<pad name="1" x="-3.9688" y="0" drill="0.8128" diameter="1.778" shape="octagon"/>
<pad name="2" x="3.9688" y="0" drill="0.8128" diameter="1.778" shape="octagon"/>
<text x="6.0325" y="0" size="1.27" layer="25" ratio="14">&gt;NAME</text>
<text x="6.0325" y="-1.905" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C072-095X060">
<description>&lt;b&gt;NP FILM CAP&lt;/b&gt;&lt;p&gt;
7.2 mm lead spacing&lt;br&gt; outline 9.5 x 6.0 mm&lt;br&gt; grid 0.0125 inch</description>
<wire x1="-0.3175" y1="0.9525" x2="-0.3175" y2="0" width="0.254" layer="51"/>
<wire x1="-0.3175" y1="0" x2="-0.3175" y2="-0.9525" width="0.254" layer="51"/>
<wire x1="0.3175" y1="0.9525" x2="0.3175" y2="0" width="0.254" layer="51"/>
<wire x1="0.3175" y1="0" x2="0.3175" y2="-0.9525" width="0.254" layer="51"/>
<wire x1="-0.3175" y1="0" x2="-3.56" y2="0" width="0.2032" layer="51"/>
<wire x1="0.3175" y1="0" x2="3.56" y2="0" width="0.2032" layer="51"/>
<wire x1="4.195" y1="-3.0638" x2="4.83" y2="-2.4288" width="0.2032" layer="21" curve="90"/>
<wire x1="4.83" y1="-2.4288" x2="4.83" y2="2.4287" width="0.2032" layer="21"/>
<wire x1="4.83" y1="2.4287" x2="4.195" y2="3.0637" width="0.2032" layer="21" curve="90"/>
<wire x1="4.195" y1="3.0637" x2="-4.195" y2="3.0637" width="0.2032" layer="21"/>
<wire x1="-4.195" y1="3.0637" x2="-4.83" y2="2.4287" width="0.2032" layer="21" curve="90"/>
<wire x1="-4.83" y1="2.4287" x2="-4.83" y2="-2.4288" width="0.2032" layer="21"/>
<wire x1="-4.83" y1="-2.4288" x2="-4.195" y2="-3.0638" width="0.2032" layer="21" curve="90"/>
<wire x1="-4.195" y1="-3.0638" x2="4.195" y2="-3.0638" width="0.2032" layer="21"/>
<pad name="1" x="-3.6" y="0" drill="0.8128" diameter="1.778" shape="octagon"/>
<pad name="2" x="3.6" y="0" drill="0.8128" diameter="1.778" shape="octagon"/>
<text x="5.5325" y="-1.405" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
<text x="5.5325" y="0.5" size="1.27" layer="25" ratio="14">&gt;NAME</text>
</package>
<package name="C072-095X065">
<description>&lt;b&gt;NP FILM CAP&lt;/b&gt;&lt;p&gt;
7.2 mm lead spacing&lt;br&gt; outline 9.5 x 6.5 mm&lt;br&gt; grid 0.0125 inch</description>
<wire x1="-0.3175" y1="0.9525" x2="-0.3175" y2="0" width="0.254" layer="51"/>
<wire x1="-0.3175" y1="0" x2="-0.3175" y2="-0.9525" width="0.254" layer="51"/>
<wire x1="0.3175" y1="0.9525" x2="0.3175" y2="0" width="0.254" layer="51"/>
<wire x1="0.3175" y1="0" x2="0.3175" y2="-0.9525" width="0.254" layer="51"/>
<wire x1="-0.3175" y1="0" x2="-3.56" y2="0" width="0.2032" layer="51"/>
<wire x1="0.3175" y1="0" x2="3.56" y2="0" width="0.2032" layer="51"/>
<wire x1="4.195" y1="-3.3138" x2="4.83" y2="-2.6788" width="0.2032" layer="21" curve="90"/>
<wire x1="4.83" y1="-2.6788" x2="4.83" y2="2.6787" width="0.2032" layer="21"/>
<wire x1="4.83" y1="2.6787" x2="4.195" y2="3.3137" width="0.2032" layer="21" curve="90"/>
<wire x1="4.195" y1="3.3137" x2="-4.195" y2="3.3137" width="0.2032" layer="21"/>
<wire x1="-4.195" y1="3.3137" x2="-4.83" y2="2.6787" width="0.2032" layer="21" curve="90"/>
<wire x1="-4.83" y1="2.6787" x2="-4.83" y2="-2.6788" width="0.2032" layer="21"/>
<wire x1="-4.83" y1="-2.6788" x2="-4.195" y2="-3.3138" width="0.2032" layer="21" curve="90"/>
<wire x1="-4.195" y1="-3.3138" x2="4.195" y2="-3.3138" width="0.2032" layer="21"/>
<pad name="1" x="-3.6" y="0" drill="0.8128" diameter="1.778" shape="octagon"/>
<pad name="2" x="3.6" y="0" drill="0.8128" diameter="1.778" shape="octagon"/>
<text x="5.5325" y="-1.405" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
<text x="5.5325" y="0.5" size="1.27" layer="25" ratio="14">&gt;NAME</text>
</package>
<package name="C072-095X070">
<description>&lt;b&gt;NP FILM CAP&lt;/b&gt;&lt;p&gt;
7.2 mm lead spacing&lt;br&gt; outline 9.5 x 7.0 mm&lt;br&gt; grid 0.0125 inch</description>
<wire x1="-0.3175" y1="0.9525" x2="-0.3175" y2="0" width="0.254" layer="51"/>
<wire x1="-0.3175" y1="0" x2="-0.3175" y2="-0.9525" width="0.254" layer="51"/>
<wire x1="0.3175" y1="0.9525" x2="0.3175" y2="0" width="0.254" layer="51"/>
<wire x1="0.3175" y1="0" x2="0.3175" y2="-0.9525" width="0.254" layer="51"/>
<wire x1="-0.3175" y1="0" x2="-3.56" y2="0" width="0.2032" layer="51"/>
<wire x1="0.3175" y1="0" x2="3.56" y2="0" width="0.2032" layer="51"/>
<wire x1="4.195" y1="-3.5638" x2="4.83" y2="-2.9288" width="0.2032" layer="21" curve="90"/>
<wire x1="4.83" y1="-2.9288" x2="4.83" y2="2.9287" width="0.2032" layer="21"/>
<wire x1="4.83" y1="2.9287" x2="4.195" y2="3.5637" width="0.2032" layer="21" curve="90"/>
<wire x1="4.195" y1="3.5637" x2="-4.195" y2="3.5637" width="0.2032" layer="21"/>
<wire x1="-4.195" y1="3.5637" x2="-4.83" y2="2.9287" width="0.2032" layer="21" curve="90"/>
<wire x1="-4.83" y1="2.9287" x2="-4.83" y2="-2.9288" width="0.2032" layer="21"/>
<wire x1="-4.83" y1="-2.9288" x2="-4.195" y2="-3.5638" width="0.2032" layer="21" curve="90"/>
<wire x1="-4.195" y1="-3.5638" x2="4.195" y2="-3.5638" width="0.2032" layer="21"/>
<pad name="1" x="-3.6" y="0" drill="0.8128" diameter="1.778" shape="octagon"/>
<pad name="2" x="3.6" y="0" drill="0.8128" diameter="1.778" shape="octagon"/>
<text x="5.5325" y="-1.405" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
<text x="5.5325" y="0.5" size="1.27" layer="25" ratio="14">&gt;NAME</text>
</package>
<package name="C025-040X018">
<description>&lt;b&gt;NP FILM CAP&lt;/b&gt;&lt;p&gt; 
2.5 mm lead spacing&lt;br&gt; outline 4.0 x 1.8 mm&lt;br&gt;grid 0.0125 inch</description>
<wire x1="0.3175" y1="0" x2="1.27" y2="0" width="0.2032" layer="51"/>
<wire x1="-0.3175" y1="0" x2="-1.27" y2="0" width="0.2032" layer="51"/>
<wire x1="-0.3175" y1="0.635" x2="-0.3175" y2="-0.635" width="0.254" layer="51"/>
<wire x1="0.3175" y1="0.635" x2="0.3175" y2="-0.635" width="0.254" layer="51"/>
<wire x1="-1.7762" y1="1.0587" x2="-2.2762" y2="0.5587" width="0.2032" layer="21" curve="90"/>
<wire x1="-2.2762" y1="0.5587" x2="-2.2762" y2="-0.5587" width="0.2032" layer="21"/>
<wire x1="-2.2762" y1="-0.5587" x2="-1.7762" y2="-1.0587" width="0.2032" layer="21" curve="90"/>
<wire x1="-1.7762" y1="-1.0587" x2="1.7762" y2="-1.0587" width="0.2032" layer="21"/>
<wire x1="1.7762" y1="-1.0587" x2="2.2762" y2="-0.5587" width="0.2032" layer="21" curve="90"/>
<wire x1="2.2762" y1="-0.5587" x2="2.2762" y2="0.5587" width="0.2032" layer="21"/>
<wire x1="2.2762" y1="0.5587" x2="1.7762" y2="1.0587" width="0.2032" layer="21" curve="90"/>
<wire x1="1.7762" y1="1.0587" x2="-1.7762" y2="1.0587" width="0.2032" layer="21"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" diameter="1.4224" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" diameter="1.4224" shape="octagon"/>
<text x="3.175" y="0" size="1.27" layer="25" ratio="14">&gt;NAME</text>
<text x="3.175" y="-1.5875" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C020-035X018">
<description>&lt;b&gt;NP FILM CAP&lt;/b&gt;&lt;p&gt; 
2 mm lead spacing&lt;br&gt; outline 3.5 x 1.8 mm&lt;br&gt;grid 0.0125 inch</description>
<wire x1="-0.3175" y1="0.535" x2="-0.3175" y2="0" width="0.254" layer="51"/>
<wire x1="-0.3175" y1="0" x2="-0.3175" y2="-0.535" width="0.254" layer="51"/>
<wire x1="0.3175" y1="0.535" x2="0.3175" y2="0" width="0.254" layer="51"/>
<wire x1="0.3175" y1="0" x2="0.3175" y2="-0.535" width="0.254" layer="51"/>
<wire x1="-0.3175" y1="0" x2="-0.9525" y2="0" width="0.2032" layer="51"/>
<wire x1="0.3175" y1="0" x2="0.9525" y2="0" width="0.2032" layer="51"/>
<wire x1="-1.3" y1="0.9" x2="-1.8" y2="0.4" width="0.2032" layer="21" curve="90"/>
<wire x1="-1.8" y1="0.4" x2="-1.8" y2="-0.4" width="0.2032" layer="21"/>
<wire x1="-1.8" y1="-0.4" x2="-1.3" y2="-0.9" width="0.2032" layer="21" curve="90"/>
<wire x1="-1.3" y1="-0.9" x2="1.3" y2="-0.9" width="0.2032" layer="21"/>
<wire x1="1.3" y1="-0.9" x2="1.8" y2="-0.4" width="0.2032" layer="21" curve="90"/>
<wire x1="1.8" y1="-0.4" x2="1.8" y2="0.4" width="0.2032" layer="21"/>
<wire x1="1.8" y1="0.4" x2="1.3" y2="0.9" width="0.2032" layer="21" curve="90"/>
<wire x1="1.3" y1="0.9" x2="-1.3" y2="0.9" width="0.2032" layer="21"/>
<pad name="1" x="-0.9525" y="0" drill="0.6096" diameter="1.016" shape="octagon"/>
<pad name="2" x="0.9525" y="0" drill="0.6096" diameter="1.016" shape="octagon"/>
<text x="2.54" y="-0.635" size="1.27" layer="25" ratio="14">&gt;NAME</text>
<text x="2.54" y="-2.2225" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C025-050X025">
<description>&lt;b&gt;NP FILM CAP&lt;/b&gt;&lt;p&gt;
2.5 mm lead spacing&lt;br&gt; outline 5 x 2.5 mm&lt;br&gt; grid 0.0125 inch</description>
<wire x1="0.3175" y1="0" x2="1.27" y2="0" width="0.2032" layer="51"/>
<wire x1="-0.3175" y1="0" x2="-1.27" y2="0" width="0.2032" layer="51"/>
<wire x1="-0.3175" y1="0.635" x2="-0.3175" y2="-0.635" width="0.254" layer="51"/>
<wire x1="0.3175" y1="0.635" x2="0.3175" y2="-0.635" width="0.254" layer="51"/>
<wire x1="2" y1="-1.3" x2="2.5" y2="-0.8" width="0.2032" layer="21" curve="90"/>
<wire x1="2.5" y1="-0.8" x2="2.5" y2="0.8" width="0.2032" layer="21"/>
<wire x1="2.5" y1="0.8" x2="2" y2="1.3" width="0.2032" layer="21" curve="90"/>
<wire x1="2" y1="1.3" x2="-2" y2="1.3" width="0.2032" layer="21"/>
<wire x1="-2" y1="1.3" x2="-2.5" y2="0.8" width="0.2032" layer="21" curve="90"/>
<wire x1="-2.5" y1="0.8" x2="-2.5" y2="-0.8" width="0.2032" layer="21"/>
<wire x1="-2.5" y1="-0.8" x2="-2" y2="-1.3" width="0.2032" layer="21" curve="90"/>
<wire x1="-2" y1="-1.3" x2="2" y2="-1.3" width="0.2032" layer="21"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" diameter="1.4224" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" diameter="1.4224" shape="octagon"/>
<text x="3.175" y="0" size="1.27" layer="25" ratio="14">&gt;NAME</text>
<text x="3.175" y="-1.5875" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="CA-100-024X044">
<description>&lt;b&gt;AXIAL FILM CAP&lt;/b&gt;&lt;p&gt;
grid 10 mm, outline 2.4 x 4.4 mm</description>
<wire x1="-0.3048" y1="0.762" x2="-0.3048" y2="0" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-0.3048" y2="-0.762" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0.762" x2="0.3302" y2="0" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="0.3302" y2="-0.762" width="0.3048" layer="21"/>
<wire x1="1.27" y1="0" x2="0.3302" y2="0" width="0.2032" layer="21"/>
<wire x1="-1.27" y1="0" x2="-0.3048" y2="0" width="0.2032" layer="21"/>
<wire x1="-1.5875" y1="-1.1113" x2="-2.2225" y2="-0.4763" width="0.2032" layer="21" curve="-90"/>
<wire x1="-2.2225" y1="-0.4763" x2="-2.2225" y2="0.4763" width="0.2032" layer="21"/>
<wire x1="-2.2225" y1="0.4763" x2="-1.5875" y2="1.1113" width="0.2032" layer="21" curve="-90"/>
<wire x1="-1.5875" y1="1.1113" x2="1.5875" y2="1.1113" width="0.2032" layer="21"/>
<wire x1="1.5875" y1="1.1113" x2="2.2225" y2="0.4763" width="0.2032" layer="21" curve="-90"/>
<wire x1="2.2225" y1="0.4763" x2="2.2225" y2="-0.4763" width="0.2032" layer="21"/>
<wire x1="2.2225" y1="-0.4763" x2="1.5875" y2="-1.1113" width="0.2032" layer="21" curve="-90"/>
<wire x1="1.5875" y1="-1.1113" x2="-1.5875" y2="-1.1113" width="0.2032" layer="21"/>
<wire x1="-4.1275" y1="0" x2="-5.08" y2="0" width="0.3048" layer="51"/>
<wire x1="5.08" y1="0" x2="4.1275" y2="0" width="0.3048" layer="51"/>
<pad name="1" x="-5.08" y="0" drill="0.8128" diameter="1.9304" shape="octagon"/>
<pad name="2" x="5.08" y="0" drill="0.8128" diameter="1.9304" shape="octagon"/>
<text x="-2.2225" y="1.5875" size="1.27" layer="25" ratio="14">&gt;NAME</text>
<text x="-2.2225" y="-2.54" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-4.1275" y1="-0.1588" x2="-2.2225" y2="0.1588" layer="21"/>
<rectangle x1="2.2225" y1="-0.1588" x2="4.1275" y2="0.1588" layer="21"/>
</package>
<package name="CA-050-024X044">
<description>&lt;b&gt;AXIAL FILM CAP&lt;/b&gt;&lt;p&gt;
grid 5 mm, outline 2.4 x 4.4 mm</description>
<wire x1="-0.3048" y1="0.762" x2="-0.3048" y2="0" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-0.3048" y2="-0.762" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0.762" x2="0.3302" y2="0" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="0.3302" y2="-0.762" width="0.3048" layer="21"/>
<wire x1="1.27" y1="0" x2="0.3302" y2="0" width="0.2032" layer="21"/>
<wire x1="-1.27" y1="0" x2="-0.3048" y2="0" width="0.2032" layer="21"/>
<wire x1="-1.5875" y1="-1.1113" x2="-2.2225" y2="-0.4763" width="0.2032" layer="21" curve="-90"/>
<wire x1="-2.2225" y1="-0.4763" x2="-2.2225" y2="0.4763" width="0.2032" layer="51"/>
<wire x1="-2.2225" y1="0.4763" x2="-1.5875" y2="1.1113" width="0.2032" layer="21" curve="-90"/>
<wire x1="-1.5875" y1="1.1113" x2="1.5875" y2="1.1113" width="0.2032" layer="21"/>
<wire x1="1.5875" y1="1.1113" x2="2.2225" y2="0.4763" width="0.2032" layer="21" curve="-90"/>
<wire x1="2.2225" y1="0.4763" x2="2.2225" y2="-0.4763" width="0.2032" layer="51"/>
<wire x1="2.2225" y1="-0.4763" x2="1.5875" y2="-1.1113" width="0.2032" layer="21" curve="-90"/>
<wire x1="1.5875" y1="-1.1113" x2="-1.5875" y2="-1.1113" width="0.2032" layer="21"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" diameter="1.778" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" diameter="1.778" shape="octagon"/>
<text x="-2.2225" y="1.5875" size="1.27" layer="25" ratio="14">&gt;NAME</text>
<text x="-2.2225" y="-2.54" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-2.5395" y1="-0.1588" x2="-2.222" y2="0.1588" layer="51"/>
<rectangle x1="2.2225" y1="-0.1588" x2="2.54" y2="0.1588" layer="51"/>
</package>
<package name="0508">
<description>&lt;b&gt;SMD CHIP CAP&lt;/b&gt;&lt;p&gt;
0508 reverse geometry</description>
<wire x1="-0.381" y1="1.06" x2="0.381" y2="1.06" width="0.1016" layer="51"/>
<wire x1="-0.356" y1="-1.06" x2="0.381" y2="-1.06" width="0.1016" layer="51"/>
<smd name="1" x="-0.7025" y="0" dx="0.9" dy="2" layer="1"/>
<smd name="2" x="0.7025" y="0" dx="0.9" dy="2" layer="1"/>
<text x="-1.1525" y="1.3725" size="0.4064" layer="25" ratio="14">&gt;NAME</text>
<text x="-1.1425" y="-1.605" size="0.4064" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-0.8416" y1="-1.1" x2="-0.4" y2="1.1" layer="51"/>
<rectangle x1="0.4" y1="-1.1" x2="0.8557" y2="1.1" layer="51"/>
<rectangle x1="-0.1588" y1="-1" x2="0.1588" y2="1" layer="35"/>
</package>
<package name="0612">
<description>&lt;b&gt;SMD CHIP CAP&lt;/b&gt;&lt;p&gt;
0612 reverse geometry</description>
<wire x1="-0.965" y1="1.387" x2="0.965" y2="1.387" width="0.1016" layer="51"/>
<wire x1="-0.965" y1="-1.387" x2="0.965" y2="-1.387" width="0.1016" layer="51"/>
<smd name="P$1" x="-0.8875" y="0" dx="0.9" dy="3.05" layer="1"/>
<smd name="P$2" x="0.7875" y="0" dx="0.9" dy="3.05" layer="1"/>
<text x="-1.0875" y="1.9113" size="0.4064" layer="25" ratio="14">&gt;NAME</text>
<text x="-1.1875" y="-2.1138" size="0.4064" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-1.1012" y1="-1.4" x2="-0.3511" y2="1.4" layer="51"/>
<rectangle x1="0.2517" y1="-1.4" x2="1.0018" y2="1.4" layer="51"/>
<rectangle x1="-0.3" y1="-1.5" x2="0.2" y2="1.5" layer="35"/>
</package>
</packages>
<symbols>
<symbol name="CNP-">
<wire x1="-2.54" y1="0" x2="-0.635" y2="0" width="0.1524" layer="94"/>
<wire x1="2.54" y1="0" x2="0.635" y2="0" width="0.1524" layer="94"/>
<wire x1="0.635" y1="1.5875" x2="0.635" y2="0" width="0.4064" layer="94"/>
<wire x1="0.635" y1="0" x2="0.635" y2="-1.5875" width="0.4064" layer="94"/>
<wire x1="-0.635" y1="1.5875" x2="-0.635" y2="0" width="0.4064" layer="94"/>
<wire x1="-0.635" y1="0" x2="-0.635" y2="-1.5875" width="0.4064" layer="94"/>
<text x="1.905" y="0.635" size="1.778" layer="95">&gt;NAME</text>
<text x="-5.08" y="-4.1275" size="1.778" layer="96">&gt;VALUE</text>
<text x="-2.794" y="-1.27" size="0.8636" layer="93">1</text>
<text x="2.286" y="-1.27" size="0.8636" layer="93">2</text>
<pin name="1" x="-2.54" y="0" visible="off" length="point" direction="pas" swaplevel="1"/>
<pin name="2" x="2.54" y="0" visible="off" length="point" direction="pas" swaplevel="1" rot="R180"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="CNP-" prefix="C" uservalue="yes">
<description>&lt;b&gt;NON-POLARIZED CAP&lt;/b&gt;</description>
<gates>
<gate name="G$1" symbol="CNP-" x="2.54" y="0"/>
</gates>
<devices>
<device name="0402" package="0402">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0603" package="0603">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0805" package="0805">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="1206" package="1206">
<connects>
<connect gate="G$1" pin="1" pad="P$1"/>
<connect gate="G$1" pin="2" pad="P$2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="1210" package="1210">
<connects>
<connect gate="G$1" pin="1" pad="P$1"/>
<connect gate="G$1" pin="2" pad="P$2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="1812" package="1812">
<connects>
<connect gate="G$1" pin="1" pad="P$1"/>
<connect gate="G$1" pin="2" pad="P$2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="1812K" package="1812K">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="1825" package="1825">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="1825K" package="1825K">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="2012" package="2012">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="2220K" package="2220K">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="2225K" package="2225K">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="3216" package="3216">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="3225" package="3225">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="4532" package="4532">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="4564" package="4564">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C100-140X060" package="C100-140X060">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C120-150X060" package="C120-150X060">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C145-180X070" package="C145-180X070">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C150-190X070" package="C150-190X070">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C170-210X070" package="C170-210X070">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C200-230X080" package="C200-230X080">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C225-260X090" package="C225-260X090">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C275-320X100" package="C275-320X100">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C035-055X025" package="C035-055X025">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C050-075X030" package="C050-075X030">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C075-110X050" package="C075-110X050">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C085-110X050" package="C085-110X050">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C275-310X150" package="C275-310X150">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C050-075X040" package="C050-075X040">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C060-090X030" package="C060-090X030">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C050-080X045" package="C050-080X045">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C075-110X060" package="C075-110X060">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C075-110X070" package="C075-110X070">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C050-110X055" package="C050-110X055">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C075-105X035" package="C075-105X035">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C050-075X035" package="C050-075X035">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C050-075X080" package="C050-075X080">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C072-095X030" package="C072-095X030">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C072-095X035" package="C072-095X035">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C080-110X045" package="C080-110X045">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C072-095X060" package="C072-095X060">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C072-095X065" package="C072-095X065">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C072-095X070" package="C072-095X070">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C025-040X018" package="C025-040X018">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C020-035X018" package="C020-035X018">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C025-050X025" package="C025-050X025">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="CA-100-024X044" package="CA-100-024X044">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="CA-050-024X044" package="CA-050-024X044">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0508" package="0508">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0612" package="0612">
<connects>
<connect gate="G$1" pin="1" pad="P$1"/>
<connect gate="G$1" pin="2" pad="P$2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="wirepad">
<description>&lt;b&gt;Single Pads&lt;/b&gt;&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="1,6/0,8">
<description>&lt;b&gt;THROUGH-HOLE PAD&lt;/b&gt;</description>
<wire x1="-0.762" y1="0.762" x2="-0.508" y2="0.762" width="0.1524" layer="21"/>
<wire x1="-0.762" y1="0.762" x2="-0.762" y2="0.508" width="0.1524" layer="21"/>
<wire x1="0.762" y1="0.762" x2="0.762" y2="0.508" width="0.1524" layer="21"/>
<wire x1="0.762" y1="0.762" x2="0.508" y2="0.762" width="0.1524" layer="21"/>
<wire x1="0.762" y1="-0.508" x2="0.762" y2="-0.762" width="0.1524" layer="21"/>
<wire x1="0.762" y1="-0.762" x2="0.508" y2="-0.762" width="0.1524" layer="21"/>
<wire x1="-0.508" y1="-0.762" x2="-0.762" y2="-0.762" width="0.1524" layer="21"/>
<wire x1="-0.762" y1="-0.762" x2="-0.762" y2="-0.508" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="0.635" width="0.1524" layer="51"/>
<pad name="1" x="0" y="0" drill="0.8128" diameter="1.6002" shape="octagon"/>
<text x="-0.762" y="1.016" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="0" y="0.6" size="0.0254" layer="27">&gt;VALUE</text>
</package>
<package name="1,6/0,9">
<description>&lt;b&gt;THROUGH-HOLE PAD&lt;/b&gt;</description>
<wire x1="-0.508" y1="0.762" x2="-0.762" y2="0.762" width="0.1524" layer="21"/>
<wire x1="-0.762" y1="0.762" x2="-0.762" y2="0.508" width="0.1524" layer="21"/>
<wire x1="-0.762" y1="-0.508" x2="-0.762" y2="-0.762" width="0.1524" layer="21"/>
<wire x1="-0.762" y1="-0.762" x2="-0.508" y2="-0.762" width="0.1524" layer="21"/>
<wire x1="0.508" y1="-0.762" x2="0.762" y2="-0.762" width="0.1524" layer="21"/>
<wire x1="0.762" y1="-0.762" x2="0.762" y2="-0.508" width="0.1524" layer="21"/>
<wire x1="0.762" y1="0.508" x2="0.762" y2="0.762" width="0.1524" layer="21"/>
<wire x1="0.762" y1="0.762" x2="0.508" y2="0.762" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="0.635" width="0.1524" layer="51"/>
<pad name="1" x="0" y="0" drill="0.9144" diameter="1.6002" shape="octagon"/>
<text x="-0.762" y="1.016" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="0" y="0.6" size="0.0254" layer="27">&gt;VALUE</text>
</package>
<package name="2,15/1,0">
<description>&lt;b&gt;THROUGH-HOLE PAD&lt;/b&gt;</description>
<wire x1="1.143" y1="-1.143" x2="1.143" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="1.143" y1="-1.143" x2="0.635" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="1.143" y1="0.635" x2="1.143" y2="1.143" width="0.1524" layer="21"/>
<wire x1="1.143" y1="1.143" x2="0.635" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="1.143" x2="-1.143" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-1.143" y1="1.143" x2="-1.143" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-1.143" y1="-0.635" x2="-1.143" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="-1.143" y1="-1.143" x2="-0.635" y2="-1.143" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="1.016" width="0.1524" layer="51"/>
<pad name="1" x="0" y="0" drill="1.016" diameter="2.159" shape="octagon"/>
<text x="-1.143" y="1.397" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="0" y="1" size="0.0254" layer="27">&gt;VALUE</text>
</package>
<package name="2,54/0,8">
<description>&lt;b&gt;THROUGH-HOLE PAD&lt;/b&gt;</description>
<wire x1="-1.27" y1="1.27" x2="-0.762" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="1.27" x2="-1.27" y2="0.762" width="0.1524" layer="21"/>
<wire x1="1.27" y1="1.27" x2="1.27" y2="0.762" width="0.1524" layer="21"/>
<wire x1="1.27" y1="1.27" x2="0.762" y2="1.27" width="0.1524" layer="21"/>
<wire x1="1.27" y1="-0.762" x2="1.27" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="1.27" y1="-1.27" x2="0.762" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-0.762" y1="-1.27" x2="-1.27" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="-1.27" x2="-1.27" y2="-0.762" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="0.635" width="0.1524" layer="51"/>
<pad name="1" x="0" y="0" drill="0.8128" diameter="2.54" shape="octagon"/>
<text x="-1.27" y="1.524" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="0" y="0.6" size="0.0254" layer="27">&gt;VALUE</text>
</package>
<package name="2,54/0,9">
<description>&lt;b&gt;THROUGH-HOLE PAD&lt;/b&gt;</description>
<wire x1="-1.27" y1="1.27" x2="-0.762" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="1.27" x2="-1.27" y2="0.762" width="0.1524" layer="21"/>
<wire x1="1.27" y1="1.27" x2="1.27" y2="0.762" width="0.1524" layer="21"/>
<wire x1="1.27" y1="1.27" x2="0.762" y2="1.27" width="0.1524" layer="21"/>
<wire x1="1.27" y1="-0.762" x2="1.27" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="1.27" y1="-1.27" x2="0.762" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-0.762" y1="-1.27" x2="-1.27" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="-1.27" x2="-1.27" y2="-0.762" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="0.635" width="0.1524" layer="51"/>
<pad name="1" x="0" y="0" drill="0.9144" diameter="2.54" shape="octagon"/>
<text x="-1.27" y="1.524" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="0" y="0.6" size="0.0254" layer="27">&gt;VALUE</text>
</package>
<package name="2,54/1,0">
<description>&lt;b&gt;THROUGH-HOLE PAD&lt;/b&gt;</description>
<wire x1="1.27" y1="1.27" x2="1.27" y2="0.762" width="0.1524" layer="21"/>
<wire x1="1.27" y1="1.27" x2="0.762" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="1.27" x2="-0.762" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="1.27" x2="-1.27" y2="0.762" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="-1.27" x2="-1.27" y2="-0.762" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="-1.27" x2="-0.762" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="0.762" y1="-1.27" x2="1.27" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="1.27" y1="-1.27" x2="1.27" y2="-0.762" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="1.016" width="0.1524" layer="51"/>
<pad name="1" x="0" y="0" drill="1.016" diameter="2.54" shape="octagon"/>
<text x="-1.27" y="1.524" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="0" y="1" size="0.0254" layer="27">&gt;VALUE</text>
</package>
<package name="2,54/1,1">
<description>&lt;b&gt;THROUGH-HOLE PAD&lt;/b&gt;</description>
<wire x1="1.27" y1="1.27" x2="1.27" y2="0.762" width="0.1524" layer="21"/>
<wire x1="1.27" y1="1.27" x2="0.762" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="1.27" x2="-1.27" y2="0.762" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="1.27" x2="-0.762" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="-1.27" x2="-1.27" y2="-0.762" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="-1.27" x2="-0.762" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="1.27" y1="-1.27" x2="0.762" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="1.27" y1="-1.27" x2="1.27" y2="-0.762" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="1.016" width="0.1524" layer="51"/>
<pad name="1" x="0" y="0" drill="1.1176" diameter="2.54" shape="octagon"/>
<text x="-1.27" y="1.524" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="0" y="1" size="0.0254" layer="27">&gt;VALUE</text>
</package>
<package name="3,17/1,1">
<description>&lt;b&gt;THROUGH-HOLE PAD&lt;/b&gt;</description>
<wire x1="1.524" y1="-1.016" x2="1.524" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="1.524" y1="-1.524" x2="1.016" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="-1.016" y1="-1.524" x2="-1.524" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="-1.524" y1="-1.524" x2="-1.524" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-1.524" y1="1.016" x2="-1.524" y2="1.524" width="0.1524" layer="21"/>
<wire x1="-1.524" y1="1.524" x2="-1.016" y2="1.524" width="0.1524" layer="21"/>
<wire x1="1.016" y1="1.524" x2="1.524" y2="1.524" width="0.1524" layer="21"/>
<wire x1="1.524" y1="1.524" x2="1.524" y2="1.016" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="1.27" width="0.1524" layer="51"/>
<pad name="1" x="0" y="0" drill="1.1176" diameter="3.175" shape="octagon"/>
<text x="-1.524" y="1.905" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="0" y="1.2" size="0.0254" layer="27">&gt;VALUE</text>
</package>
<package name="3,17/1,2">
<description>&lt;b&gt;THROUGH-HOLE PAD&lt;/b&gt;</description>
<wire x1="1.524" y1="-1.016" x2="1.524" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="1.524" y1="-1.524" x2="1.016" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="-1.016" y1="-1.524" x2="-1.524" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="-1.524" y1="-1.524" x2="-1.524" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-1.524" y1="1.016" x2="-1.524" y2="1.524" width="0.1524" layer="21"/>
<wire x1="-1.524" y1="1.524" x2="-1.016" y2="1.524" width="0.1524" layer="21"/>
<wire x1="1.016" y1="1.524" x2="1.524" y2="1.524" width="0.1524" layer="21"/>
<wire x1="1.524" y1="1.524" x2="1.524" y2="1.016" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="1.27" width="0.1524" layer="51"/>
<pad name="1" x="0" y="0" drill="1.1938" diameter="3.175" shape="octagon"/>
<text x="-1.524" y="1.905" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="0" y="1.2" size="0.0254" layer="27">&gt;VALUE</text>
</package>
<package name="3,17/1,3">
<description>&lt;b&gt;THROUGH-HOLE PAD&lt;/b&gt;</description>
<wire x1="1.524" y1="-1.016" x2="1.524" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="1.524" y1="-1.524" x2="1.016" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="-1.016" y1="-1.524" x2="-1.524" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="-1.524" y1="-1.524" x2="-1.524" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-1.524" y1="1.016" x2="-1.524" y2="1.524" width="0.1524" layer="21"/>
<wire x1="-1.524" y1="1.524" x2="-1.016" y2="1.524" width="0.1524" layer="21"/>
<wire x1="1.016" y1="1.524" x2="1.524" y2="1.524" width="0.1524" layer="21"/>
<wire x1="1.524" y1="1.524" x2="1.524" y2="1.016" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="1.27" width="0.1524" layer="51"/>
<pad name="1" x="0" y="0" drill="1.3208" diameter="3.175" shape="octagon"/>
<text x="-1.524" y="1.905" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="0" y="1.2" size="0.0254" layer="27">&gt;VALUE</text>
</package>
<package name="3,81/1,1">
<description>&lt;b&gt;THROUGH-HOLE PAD&lt;/b&gt;</description>
<wire x1="1.905" y1="-1.27" x2="1.905" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="1.905" y1="-1.905" x2="1.27" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="-1.905" x2="-1.905" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="-1.905" x2="-1.905" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="1.27" x2="-1.905" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="1.905" x2="-1.27" y2="1.905" width="0.1524" layer="21"/>
<wire x1="1.27" y1="1.905" x2="1.905" y2="1.905" width="0.1524" layer="21"/>
<wire x1="1.905" y1="1.905" x2="1.905" y2="1.27" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="1.27" width="0.1524" layer="51"/>
<pad name="1" x="0" y="0" drill="1.1176" diameter="3.81" shape="octagon"/>
<text x="-1.905" y="2.286" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="0" y="1.2" size="0.0254" layer="27">&gt;VALUE</text>
</package>
<package name="3,81/1,3">
<description>&lt;b&gt;THROUGH-HOLE PAD&lt;/b&gt;</description>
<wire x1="1.905" y1="-1.27" x2="1.905" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="1.905" y1="-1.905" x2="1.27" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="-1.905" x2="-1.905" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="-1.905" x2="-1.905" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="1.27" x2="-1.905" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="1.905" x2="-1.27" y2="1.905" width="0.1524" layer="21"/>
<wire x1="1.27" y1="1.905" x2="1.905" y2="1.905" width="0.1524" layer="21"/>
<wire x1="1.905" y1="1.905" x2="1.905" y2="1.27" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="1.27" width="0.1524" layer="51"/>
<pad name="1" x="0" y="0" drill="1.3208" diameter="3.81" shape="octagon"/>
<text x="-1.905" y="2.286" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="0" y="1.2" size="0.0254" layer="27">&gt;VALUE</text>
</package>
<package name="3,81/1,4">
<description>&lt;b&gt;THROUGH-HOLE PAD&lt;/b&gt;</description>
<wire x1="1.905" y1="-1.27" x2="1.905" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="1.905" y1="-1.905" x2="1.27" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="-1.905" x2="-1.905" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="-1.905" x2="-1.905" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="1.27" x2="-1.905" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="1.905" x2="-1.27" y2="1.905" width="0.1524" layer="21"/>
<wire x1="1.27" y1="1.905" x2="1.905" y2="1.905" width="0.1524" layer="21"/>
<wire x1="1.905" y1="1.905" x2="1.905" y2="1.27" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="1.27" width="0.1524" layer="51"/>
<pad name="1" x="0" y="0" drill="1.397" diameter="3.81" shape="octagon"/>
<text x="-1.905" y="2.286" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="0" y="1.2" size="0.0254" layer="27">&gt;VALUE</text>
</package>
<package name="4,16O1,6">
<description>&lt;b&gt;THROUGH-HOLE PAD&lt;/b&gt;</description>
<pad name="1" x="0" y="0" drill="1.6002" diameter="4.1656" shape="octagon"/>
<text x="0" y="0" size="0.0254" layer="27">&gt;VALUE</text>
<text x="-2.1" y="2.2" size="1.27" layer="25">&gt;NAME</text>
</package>
<package name="5-1,8">
<description>&lt;b&gt;THROUGH-HOLE PAD&lt;/b&gt;</description>
<wire x1="1.1684" y1="2.794" x2="-1.1684" y2="2.794" width="0.1524" layer="21"/>
<wire x1="-1.1684" y1="-2.794" x2="-1.1684" y2="2.794" width="0.1524" layer="21"/>
<wire x1="-1.1684" y1="-2.794" x2="1.1684" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="1.1684" y1="2.794" x2="1.1684" y2="-2.794" width="0.1524" layer="21"/>
<smd name="1" x="0" y="0" dx="1.8288" dy="5.08" layer="1"/>
<text x="-1.524" y="-2.54" size="1.27" layer="25" ratio="10" rot="R90">&gt;NAME</text>
<text x="-0.1" y="2.8" size="0.0254" layer="27">&gt;VALUE</text>
</package>
<package name="5-2,5">
<description>&lt;b&gt;THROUGH-HOLE PAD&lt;/b&gt;</description>
<wire x1="1.524" y1="2.794" x2="-1.524" y2="2.794" width="0.1524" layer="21"/>
<wire x1="-1.524" y1="-2.794" x2="-1.524" y2="2.794" width="0.1524" layer="21"/>
<wire x1="-1.524" y1="-2.794" x2="1.524" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="1.524" y1="2.794" x2="1.524" y2="-2.794" width="0.1524" layer="21"/>
<smd name="1" x="0" y="0" dx="2.54" dy="5.08" layer="1"/>
<text x="-1.778" y="-2.54" size="1.27" layer="25" ratio="10" rot="R90">&gt;NAME</text>
<text x="-0.1" y="2.8" size="0.0254" layer="27">&gt;VALUE</text>
</package>
<package name="SMD1,27-2,54">
<description>&lt;b&gt;SMD PAD&lt;/b&gt;</description>
<smd name="1" x="0" y="0" dx="1.27" dy="2.54" layer="1"/>
<text x="0" y="0" size="0.0254" layer="27">&gt;VALUE</text>
<text x="-0.8" y="-2.4" size="1.27" layer="25" rot="R90">&gt;NAME</text>
</package>
<package name="SMD2,54-5,08">
<description>&lt;b&gt;SMD PAD&lt;/b&gt;</description>
<smd name="1" x="0" y="0" dx="2.54" dy="5.08" layer="1"/>
<text x="0" y="0" size="0.0254" layer="27">&gt;VALUE</text>
<text x="-1.5" y="-2.5" size="1.27" layer="25" rot="R90">&gt;NAME</text>
</package>
</packages>
<symbols>
<symbol name="PAD">
<wire x1="-1.016" y1="1.016" x2="1.016" y2="-1.016" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-1.016" x2="1.016" y2="1.016" width="0.254" layer="94"/>
<text x="-1.143" y="1.8542" size="1.778" layer="95">&gt;NAME</text>
<text x="-1.143" y="-3.302" size="1.778" layer="96">&gt;VALUE</text>
<pin name="P" x="2.54" y="0" visible="off" length="short" direction="pas" rot="R180"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="WIREPAD" prefix="PAD">
<description>&lt;b&gt;Wire PAD&lt;/b&gt; connect wire on PCB</description>
<gates>
<gate name="G$1" symbol="PAD" x="0" y="0"/>
</gates>
<devices>
<device name="1,6/0,8" package="1,6/0,8">
<connects>
<connect gate="G$1" pin="P" pad="1"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="1,6/0,9" package="1,6/0,9">
<connects>
<connect gate="G$1" pin="P" pad="1"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="2,15/1,0" package="2,15/1,0">
<connects>
<connect gate="G$1" pin="P" pad="1"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="2,54/0,8" package="2,54/0,8">
<connects>
<connect gate="G$1" pin="P" pad="1"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="2,54/0,9" package="2,54/0,9">
<connects>
<connect gate="G$1" pin="P" pad="1"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="2,54/1,0" package="2,54/1,0">
<connects>
<connect gate="G$1" pin="P" pad="1"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="2,54/1,1" package="2,54/1,1">
<connects>
<connect gate="G$1" pin="P" pad="1"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="3,17/1,1" package="3,17/1,1">
<connects>
<connect gate="G$1" pin="P" pad="1"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="3,17/1,2" package="3,17/1,2">
<connects>
<connect gate="G$1" pin="P" pad="1"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="3,17/1,3" package="3,17/1,3">
<connects>
<connect gate="G$1" pin="P" pad="1"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="3,81/1,1" package="3,81/1,1">
<connects>
<connect gate="G$1" pin="P" pad="1"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="3,81/1,3" package="3,81/1,3">
<connects>
<connect gate="G$1" pin="P" pad="1"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="3,81/1,4" package="3,81/1,4">
<connects>
<connect gate="G$1" pin="P" pad="1"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="4,16O1,6" package="4,16O1,6">
<connects>
<connect gate="G$1" pin="P" pad="1"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="SMD5-1,8" package="5-1,8">
<connects>
<connect gate="G$1" pin="P" pad="1"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="SMD5-2,5" package="5-2,5">
<connects>
<connect gate="G$1" pin="P" pad="1"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="SMD1,27-254" package="SMD1,27-2,54">
<connects>
<connect gate="G$1" pin="P" pad="1"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="SMD2,54-5,08" package="SMD2,54-5,08">
<connects>
<connect gate="G$1" pin="P" pad="1"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="SamacSys_Parts">
<description>&lt;b&gt;https://componentsearchengine.com&lt;/b&gt;&lt;p&gt;
&lt;author&gt;Created by SamacSys&lt;/author&gt;</description>
<packages>
<package name="LEDC2012X80N">
<description>&lt;b&gt;150080BS75000&lt;/b&gt;&lt;br&gt;
</description>
<smd name="1" x="-1.05" y="0" dx="1.2" dy="1.3" layer="1" rot="R90"/>
<smd name="2" x="1.05" y="0" dx="1.2" dy="1.3" layer="1" rot="R90"/>
<text x="0" y="-2.54" size="1.27" layer="25" align="center">&gt;NAME</text>
<text x="0" y="2.54" size="1.27" layer="27" align="center">&gt;VALUE</text>
<wire x1="-1.85" y1="1.15" x2="1.85" y2="1.15" width="0.05" layer="51"/>
<wire x1="1.85" y1="1.15" x2="1.85" y2="-1.15" width="0.05" layer="51"/>
<wire x1="1.85" y1="-1.15" x2="-1.85" y2="-1.15" width="0.05" layer="51"/>
<wire x1="-1.85" y1="-1.15" x2="-1.85" y2="1.15" width="0.05" layer="51"/>
<wire x1="-1" y1="0.625" x2="1" y2="0.625" width="0.1" layer="51"/>
<wire x1="1" y1="0.625" x2="1" y2="-0.625" width="0.1" layer="51"/>
<wire x1="1" y1="-0.625" x2="-1" y2="-0.625" width="0.1" layer="51"/>
<wire x1="-1" y1="-0.625" x2="-1" y2="0.625" width="0.1" layer="51"/>
<wire x1="-1" y1="0.208" x2="-0.583" y2="0.625" width="0.1" layer="51"/>
<wire x1="0.95" y1="1.05" x2="-1.75" y2="1.05" width="0.2" layer="21"/>
<wire x1="-1.75" y1="1.05" x2="-1.75" y2="-1.05" width="0.2" layer="21"/>
<wire x1="-1.75" y1="-1.05" x2="0.95" y2="-1.05" width="0.2" layer="21"/>
</package>
<package name="TSX-3225V">
<description>&lt;b&gt;TSX-3225V&lt;/b&gt;&lt;br&gt;</description>
<smd name="1" x="-1.1" y="-0.8" dx="1.4" dy="1.15" layer="1"/>
<smd name="2" x="1.1" y="-0.8" dx="1.4" dy="1.15" layer="1"/>
<smd name="3" x="1.1" y="0.8" dx="1.4" dy="1.15" layer="1"/>
<smd name="4" x="-1.1" y="0.8" dx="1.4" dy="1.15" layer="1"/>
<text x="-0.43535" y="-1.2066" size="1.27" layer="25" align="center">&gt;NAME</text>
<text x="-0.43535" y="1.3334" size="1.27" layer="27" align="center">&gt;VALUE</text>
<wire x1="-1.6" y1="1.25" x2="1.6" y2="1.25" width="0.2" layer="51"/>
<wire x1="1.6" y1="1.25" x2="1.6" y2="-1.25" width="0.2" layer="51"/>
<wire x1="1.6" y1="-1.25" x2="-1.6" y2="-1.25" width="0.2" layer="51"/>
<wire x1="-1.6" y1="-1.25" x2="-1.6" y2="1.25" width="0.2" layer="51"/>
<circle x="-1.223" y="-1.965" radius="0.118" width="0.4" layer="25"/>
<wire x1="-0.25" y1="1.25" x2="0.25" y2="1.25" width="0.2" layer="21"/>
<wire x1="-1.6" y1="0.15" x2="-1.6" y2="-0.15" width="0.2" layer="21"/>
<wire x1="-0.25" y1="-1.25" x2="0.25" y2="-1.25" width="0.2" layer="21"/>
<wire x1="1.6" y1="0.15" x2="1.6" y2="-0.15" width="0.2" layer="21"/>
<wire x1="-0.31" y1="-0.16" x2="-0.01" y2="0.47" width="0.127" layer="25"/>
<wire x1="-0.01" y1="0.47" x2="0.31" y2="-0.16" width="0.127" layer="25"/>
<wire x1="-0.18" y1="0.01" x2="0.18" y2="0.01" width="0.127" layer="25"/>
</package>
<package name="SOT65P210X110-5N">
<description>&lt;b&gt;5-Lead Plastic Package (SC-70)&lt;/b&gt;&lt;br&gt;
</description>
<smd name="1" x="-1.05" y="0.65" dx="1.05" dy="0.4" layer="1"/>
<smd name="2" x="-1.05" y="0" dx="1.05" dy="0.4" layer="1"/>
<smd name="3" x="-1.05" y="-0.65" dx="1.05" dy="0.4" layer="1"/>
<smd name="4" x="1.05" y="-0.65" dx="1.05" dy="0.4" layer="1"/>
<smd name="5" x="1.05" y="0.65" dx="1.05" dy="0.4" layer="1"/>
<text x="0" y="2.54" size="1.27" layer="25" align="center">&gt;NAME</text>
<text x="0" y="-2.54" size="1.27" layer="27" align="center">&gt;VALUE</text>
<wire x1="-1.825" y1="1.35" x2="1.825" y2="1.35" width="0.05" layer="51"/>
<wire x1="1.825" y1="1.35" x2="1.825" y2="-1.35" width="0.05" layer="51"/>
<wire x1="1.825" y1="-1.35" x2="-1.825" y2="-1.35" width="0.05" layer="51"/>
<wire x1="-1.825" y1="-1.35" x2="-1.825" y2="1.35" width="0.05" layer="51"/>
<wire x1="-0.625" y1="1" x2="0.625" y2="1" width="0.1" layer="51"/>
<wire x1="0.625" y1="1" x2="0.625" y2="-1" width="0.1" layer="51"/>
<wire x1="0.625" y1="-1" x2="-0.625" y2="-1" width="0.1" layer="51"/>
<wire x1="-0.625" y1="-1" x2="-0.625" y2="1" width="0.1" layer="51"/>
<wire x1="-0.625" y1="0.35" x2="0.025" y2="1" width="0.1" layer="51"/>
<wire x1="-0.175" y1="1" x2="0.175" y2="1" width="0.2" layer="21"/>
<wire x1="0.175" y1="1" x2="0.175" y2="-1" width="0.2" layer="21"/>
<wire x1="0.175" y1="-1" x2="-0.175" y2="-1" width="0.2" layer="21"/>
<wire x1="-0.175" y1="-1" x2="-0.175" y2="1" width="0.2" layer="21"/>
<wire x1="-1.575" y1="1.1" x2="-0.525" y2="1.1" width="0.2" layer="21"/>
</package>
<package name="FUSC3216X75N">
<description>&lt;b&gt;0ZCJ0025FF2E&lt;/b&gt;&lt;br&gt;
</description>
<smd name="1" x="-1.52" y="0" dx="1.82" dy="1.17" layer="1" rot="R90"/>
<smd name="2" x="1.52" y="0" dx="1.82" dy="1.17" layer="1" rot="R90"/>
<text x="0" y="1.905" size="0.8128" layer="25" align="center">&gt;NAME</text>
<text x="0" y="-1.905" size="0.8128" layer="27" align="center">&gt;VALUE</text>
<wire x1="-2.355" y1="1.16" x2="2.355" y2="1.16" width="0.05" layer="51"/>
<wire x1="2.355" y1="1.16" x2="2.355" y2="-1.16" width="0.05" layer="51"/>
<wire x1="2.355" y1="-1.16" x2="-2.355" y2="-1.16" width="0.05" layer="51"/>
<wire x1="-2.355" y1="-1.16" x2="-2.355" y2="1.16" width="0.05" layer="51"/>
<wire x1="-1.625" y1="0.825" x2="1.625" y2="0.825" width="0.1" layer="51"/>
<wire x1="1.625" y1="0.825" x2="1.625" y2="-0.825" width="0.1" layer="51"/>
<wire x1="1.625" y1="-0.825" x2="-1.625" y2="-0.825" width="0.1" layer="51"/>
<wire x1="-1.625" y1="-0.825" x2="-1.625" y2="0.825" width="0.1" layer="51"/>
<wire x1="0" y1="0.725" x2="0" y2="-0.725" width="0.2" layer="21"/>
</package>
<package name="KMR621NG">
<description>&lt;b&gt;kmr6&lt;/b&gt;&lt;br&gt;
</description>
<smd name="1" x="-2.05" y="-0.8" dx="1" dy="0.9" layer="1" rot="R90"/>
<smd name="2" x="2.05" y="-0.8" dx="1" dy="0.9" layer="1" rot="R90"/>
<smd name="3" x="2.05" y="0.8" dx="1" dy="0.9" layer="1" rot="R90"/>
<smd name="4" x="-2.05" y="0.8" dx="1" dy="0.9" layer="1" rot="R90"/>
<text x="0" y="-1.27" size="1.27" layer="25" align="center">&gt;NAME</text>
<text x="0" y="1.27" size="1.27" layer="27" align="center">&gt;VALUE</text>
<wire x1="-2.1" y1="1.4" x2="2.1" y2="1.4" width="0.2" layer="51"/>
<wire x1="2.1" y1="1.4" x2="2.1" y2="-1.4" width="0.2" layer="51"/>
<wire x1="2.1" y1="-1.4" x2="-2.1" y2="-1.4" width="0.2" layer="51"/>
<wire x1="-2.1" y1="-1.4" x2="-2.1" y2="1.4" width="0.2" layer="51"/>
<wire x1="-1.285" y1="1.4" x2="1.285" y2="1.4" width="0.2" layer="21"/>
<wire x1="-1.285" y1="-1.4" x2="1.285" y2="-1.4" width="0.2" layer="21"/>
</package>
<package name="SOP65P210X110-6N">
<description>&lt;b&gt;SOT363&lt;/b&gt;&lt;br&gt;
</description>
<smd name="1" x="-1.062" y="0.65" dx="0.875" dy="0.4" layer="1"/>
<smd name="2" x="-1.062" y="0" dx="0.875" dy="0.4" layer="1"/>
<smd name="3" x="-1.062" y="-0.65" dx="0.875" dy="0.4" layer="1"/>
<smd name="4" x="1.062" y="-0.65" dx="0.875" dy="0.4" layer="1"/>
<smd name="5" x="1.062" y="0" dx="0.875" dy="0.4" layer="1"/>
<smd name="6" x="1.062" y="0.65" dx="0.875" dy="0.4" layer="1"/>
<text x="0" y="-2.54" size="1.27" layer="25" align="center">&gt;NAME</text>
<text x="0" y="2.54" size="1.27" layer="27" align="center">&gt;VALUE</text>
<wire x1="-1.75" y1="1.35" x2="1.75" y2="1.35" width="0.05" layer="51"/>
<wire x1="1.75" y1="1.35" x2="1.75" y2="-1.35" width="0.05" layer="51"/>
<wire x1="1.75" y1="-1.35" x2="-1.75" y2="-1.35" width="0.05" layer="51"/>
<wire x1="-1.75" y1="-1.35" x2="-1.75" y2="1.35" width="0.05" layer="51"/>
<wire x1="-0.625" y1="1" x2="0.625" y2="1" width="0.1" layer="51"/>
<wire x1="0.625" y1="1" x2="0.625" y2="-1" width="0.1" layer="51"/>
<wire x1="0.625" y1="-1" x2="-0.625" y2="-1" width="0.1" layer="51"/>
<wire x1="-0.625" y1="-1" x2="-0.625" y2="1" width="0.1" layer="51"/>
<wire x1="-0.625" y1="0.35" x2="0.025" y2="1" width="0.1" layer="51"/>
<wire x1="-0.275" y1="1" x2="0.275" y2="1" width="0.2" layer="21"/>
<wire x1="0.275" y1="1" x2="0.275" y2="-1" width="0.2" layer="21"/>
<wire x1="0.275" y1="-1" x2="-0.275" y2="-1" width="0.2" layer="21"/>
<wire x1="-0.275" y1="-1" x2="-0.275" y2="1" width="0.2" layer="21"/>
<wire x1="-1.5" y1="1.225" x2="-0.625" y2="1.225" width="0.2" layer="21"/>
</package>
</packages>
<symbols>
<symbol name="150080RS75000">
<wire x1="5.08" y1="0" x2="10.16" y2="2.54" width="0.254" layer="94"/>
<wire x1="10.16" y1="-2.54" x2="10.16" y2="2.54" width="0.254" layer="94"/>
<wire x1="10.16" y1="-2.54" x2="5.08" y2="0" width="0.254" layer="94"/>
<wire x1="5.08" y1="2.54" x2="5.08" y2="-2.54" width="0.254" layer="94"/>
<wire x1="6.35" y1="2.54" x2="3.81" y2="5.08" width="0.254" layer="94"/>
<wire x1="8.89" y1="2.54" x2="6.35" y2="5.08" width="0.254" layer="94"/>
<wire x1="2.54" y1="0" x2="5.08" y2="0" width="0.254" layer="94"/>
<wire x1="10.16" y1="0" x2="12.7" y2="0" width="0.254" layer="94"/>
<text x="12.7" y="8.89" size="1.778" layer="95">&gt;NAME</text>
<text x="12.7" y="6.35" size="1.778" layer="96">&gt;VALUE</text>
<pin name="K" x="0" y="0" length="short"/>
<pin name="A" x="15.24" y="0" length="short" rot="R180"/>
<polygon width="0.254" layer="94">
<vertex x="5.334" y="4.318"/>
<vertex x="4.572" y="3.556"/>
<vertex x="3.81" y="5.08"/>
</polygon>
<polygon width="0.254" layer="94">
<vertex x="7.874" y="4.318"/>
<vertex x="7.112" y="3.556"/>
<vertex x="6.35" y="5.08"/>
</polygon>
</symbol>
<symbol name="TSX-3225_16.0000MF18X-AC3">
<wire x1="5.08" y1="2.54" x2="25.4" y2="2.54" width="0.254" layer="94"/>
<wire x1="25.4" y1="-5.08" x2="25.4" y2="2.54" width="0.254" layer="94"/>
<wire x1="25.4" y1="-5.08" x2="5.08" y2="-5.08" width="0.254" layer="94"/>
<wire x1="5.08" y1="2.54" x2="5.08" y2="-5.08" width="0.254" layer="94"/>
<text x="26.67" y="7.62" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="26.67" y="5.08" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="1" x="0" y="-2.54" length="middle"/>
<pin name="GND_1" x="30.48" y="-2.54" length="middle" rot="R180"/>
<pin name="3" x="30.48" y="0" length="middle" rot="R180"/>
<pin name="GND_2" x="0" y="0" length="middle"/>
</symbol>
<symbol name="MIC5365-3.0YC5_TR">
<wire x1="5.08" y1="2.54" x2="22.86" y2="2.54" width="0.254" layer="94"/>
<wire x1="22.86" y1="-7.62" x2="22.86" y2="2.54" width="0.254" layer="94"/>
<wire x1="22.86" y1="-7.62" x2="5.08" y2="-7.62" width="0.254" layer="94"/>
<wire x1="5.08" y1="2.54" x2="5.08" y2="-7.62" width="0.254" layer="94"/>
<text x="24.13" y="7.62" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="24.13" y="5.08" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="VIN" x="0" y="0" length="middle"/>
<pin name="GND" x="0" y="-2.54" length="middle"/>
<pin name="EN" x="0" y="-5.08" length="middle"/>
<pin name="NC" x="27.94" y="0" length="middle" direction="nc" rot="R180"/>
<pin name="VOUT" x="27.94" y="-2.54" length="middle" rot="R180"/>
</symbol>
<symbol name="0ZCJ0025FF2E">
<wire x1="5.08" y1="1.27" x2="12.7" y2="1.27" width="0.254" layer="94"/>
<wire x1="12.7" y1="-1.27" x2="12.7" y2="1.27" width="0.254" layer="94"/>
<wire x1="12.7" y1="-1.27" x2="5.08" y2="-1.27" width="0.254" layer="94"/>
<wire x1="5.08" y1="1.27" x2="5.08" y2="-1.27" width="0.254" layer="94"/>
<wire x1="5.08" y1="0" x2="12.7" y2="0" width="0.254" layer="94"/>
<text x="13.97" y="6.35" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="13.97" y="3.81" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="1" x="0" y="0" visible="pad" length="middle"/>
<pin name="2" x="17.78" y="0" visible="pad" length="middle" rot="R180"/>
</symbol>
<symbol name="KMR631NGULCLFS">
<wire x1="5.08" y1="2.54" x2="15.24" y2="2.54" width="0.254" layer="94"/>
<wire x1="15.24" y1="-5.08" x2="15.24" y2="2.54" width="0.254" layer="94"/>
<wire x1="15.24" y1="-5.08" x2="5.08" y2="-5.08" width="0.254" layer="94"/>
<wire x1="5.08" y1="2.54" x2="5.08" y2="-5.08" width="0.254" layer="94"/>
<text x="16.51" y="7.62" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="16.51" y="5.08" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="1" x="0" y="-2.54" length="middle"/>
<pin name="2" x="20.32" y="-2.54" length="middle" rot="R180"/>
<pin name="3" x="20.32" y="0" length="middle" rot="R180"/>
<pin name="4" x="0" y="0" length="middle"/>
</symbol>
<symbol name="74LVC2G34GW,125">
<wire x1="5.08" y1="2.54" x2="20.32" y2="2.54" width="0.254" layer="94"/>
<wire x1="20.32" y1="-7.62" x2="20.32" y2="2.54" width="0.254" layer="94"/>
<wire x1="20.32" y1="-7.62" x2="5.08" y2="-7.62" width="0.254" layer="94"/>
<wire x1="5.08" y1="2.54" x2="5.08" y2="-7.62" width="0.254" layer="94"/>
<text x="21.59" y="7.62" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="21.59" y="5.08" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="1A" x="0" y="0" length="middle"/>
<pin name="GND" x="0" y="-2.54" length="middle"/>
<pin name="2A" x="0" y="-5.08" length="middle"/>
<pin name="1Y" x="25.4" y="0" length="middle" rot="R180"/>
<pin name="VCC" x="25.4" y="-2.54" length="middle" rot="R180"/>
<pin name="2Y" x="25.4" y="-5.08" length="middle" rot="R180"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="150080RS75000" prefix="LED">
<description>&lt;b&gt;LED,Wurth Elektronik,150080RS75000 Wurth Elektronik 150080RS75000, WL-SMCW Series Red LED, 625 nm 2012 (0805) Clear, Rectangle Lens SMD package&lt;/b&gt;&lt;p&gt;
Source: &lt;a href="https://componentsearchengine.com/Datasheets/2/150080RS75000.pdf"&gt; Datasheet &lt;/a&gt;</description>
<gates>
<gate name="G$1" symbol="150080RS75000" x="0" y="0"/>
</gates>
<devices>
<device name="" package="LEDC2012X80N">
<connects>
<connect gate="G$1" pin="A" pad="2"/>
<connect gate="G$1" pin="K" pad="1"/>
</connects>
<technologies>
<technology name="">
<attribute name="DESCRIPTION" value="LED,Wurth Elektronik,150080RS75000 Wurth Elektronik 150080RS75000, WL-SMCW Series Red LED, 625 nm 2012 (0805) Clear, Rectangle Lens SMD package" constant="no"/>
<attribute name="HEIGHT" value="0.8mm" constant="no"/>
<attribute name="MANUFACTURER_NAME" value="Wurth Elektronik" constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="150080RS75000" constant="no"/>
<attribute name="MOUSER_PART_NUMBER" value="710-150080RS75000" constant="no"/>
<attribute name="MOUSER_PRICE-STOCK" value="https://www.mouser.co.uk/ProductDetail/Wurth-Elektronik/150080RS75000?qs=LlUlMxKIyB2jdCo7bnfgew%3D%3D" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="TSX-3225_16.0000MF18X-AC3" prefix="Y">
<description>&lt;b&gt;MHz Range crystal unit&lt;/b&gt;&lt;p&gt;
Source: &lt;a href="https://support.epson.biz/td/api/doc_check.php?dl=brief_FA-238V_en.pdf"&gt; Datasheet &lt;/a&gt;</description>
<gates>
<gate name="G$1" symbol="TSX-3225_16.0000MF18X-AC3" x="0" y="0"/>
</gates>
<devices>
<device name="" package="TSX-3225V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="3" pad="3"/>
<connect gate="G$1" pin="GND_1" pad="2"/>
<connect gate="G$1" pin="GND_2" pad="4"/>
</connects>
<technologies>
<technology name="">
<attribute name="DESCRIPTION" value="MHz Range crystal unit" constant="no"/>
<attribute name="HEIGHT" value="mm" constant="no"/>
<attribute name="MANUFACTURER_NAME" value="Epson Timing" constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="TSX-3225 16.0000MF18X-AC3" constant="no"/>
<attribute name="MOUSER_PART_NUMBER" value="732-TX325-16F18X-AC3" constant="no"/>
<attribute name="MOUSER_PRICE-STOCK" value="https://www.mouser.co.uk/ProductDetail/Epson-Timing/TSX-3225-160000MF18X-AC3?qs=tC2qD9afNgpyT7dDEnDRhg%3D%3D" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="MIC5365-3.0YC5_TR" prefix="IC">
<description>&lt;b&gt;LDO Regulator Pos 3.0V 0.15A 5-Pin SC-70 T/R&lt;/b&gt;&lt;p&gt;
Source: &lt;a href="http://ww1.microchip.com/downloads/en/DeviceDoc/mic5365.pdf"&gt; Datasheet &lt;/a&gt;</description>
<gates>
<gate name="G$1" symbol="MIC5365-3.0YC5_TR" x="0" y="0"/>
</gates>
<devices>
<device name="" package="SOT65P210X110-5N">
<connects>
<connect gate="G$1" pin="EN" pad="3"/>
<connect gate="G$1" pin="GND" pad="2"/>
<connect gate="G$1" pin="NC" pad="4"/>
<connect gate="G$1" pin="VIN" pad="1"/>
<connect gate="G$1" pin="VOUT" pad="5"/>
</connects>
<technologies>
<technology name="">
<attribute name="DESCRIPTION" value="LDO Regulator Pos 3.0V 0.15A 5-Pin SC-70 T/R" constant="no"/>
<attribute name="HEIGHT" value="1.1mm" constant="no"/>
<attribute name="MANUFACTURER_NAME" value="Microchip" constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="MIC5365-3.0YC5 TR" constant="no"/>
<attribute name="MOUSER_PART_NUMBER" value="" constant="no"/>
<attribute name="MOUSER_PRICE-STOCK" value="" constant="no"/>
<attribute name="RS_PART_NUMBER" value="" constant="no"/>
<attribute name="RS_PRICE-STOCK" value="" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="0ZCJ0025FF2E" prefix="F">
<description>&lt;b&gt;BEL FUSE - CIRCUIT PROTECTION - 0ZCJ0025FF2E - FUSE, RESETTABLE PTC, 16VDC, 0.25A, SMD&lt;/b&gt;&lt;p&gt;
Source: &lt;a href="https://componentsearchengine.com/Datasheets/2/0ZCJ0025FF2E.pdf"&gt; Datasheet &lt;/a&gt;</description>
<gates>
<gate name="G$1" symbol="0ZCJ0025FF2E" x="0" y="0"/>
</gates>
<devices>
<device name="" package="FUSC3216X75N">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name="">
<attribute name="DESCRIPTION" value="BEL FUSE - CIRCUIT PROTECTION - 0ZCJ0025FF2E - FUSE, RESETTABLE PTC, 16VDC, 0.25A, SMD" constant="no"/>
<attribute name="HEIGHT" value="0.75mm" constant="no"/>
<attribute name="MANUFACTURER_NAME" value="Bel Circuit Protection" constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="0ZCJ0025FF2E" constant="no"/>
<attribute name="MOUSER_PART_NUMBER" value="" constant="no"/>
<attribute name="MOUSER_PRICE-STOCK" value="" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="KMR631NGULCLFS" prefix="S">
<description>&lt;b&gt;KMR631NGULCLFS (Tactile Switches)&lt;/b&gt;&lt;p&gt;
Source: &lt;a href="https://www.ckswitches.com/media/1907/kmr6.pdf"&gt; Datasheet &lt;/a&gt;</description>
<gates>
<gate name="G$1" symbol="KMR631NGULCLFS" x="0" y="0"/>
</gates>
<devices>
<device name="" package="KMR621NG">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
<connect gate="G$1" pin="4" pad="4"/>
</connects>
<technologies>
<technology name="">
<attribute name="DESCRIPTION" value="KMR631NGULCLFS (Tactile Switches)" constant="no"/>
<attribute name="HEIGHT" value="mm" constant="no"/>
<attribute name="MANUFACTURER_NAME" value="C &amp; K COMPONENTS" constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="KMR631NGULCLFS" constant="no"/>
<attribute name="MOUSER_PART_NUMBER" value="611-KMR631NGULCLFS" constant="no"/>
<attribute name="MOUSER_PRICE-STOCK" value="https://www.mouser.co.uk/ProductDetail/CK/KMR631NGULCLFS?qs=HXFqYaX1Q2xBA5gT0%2FzYzA%3D%3D" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="74LVC2G34GW,125" prefix="IC">
<description>&lt;b&gt;74LVC2G34 - Dual buffer gate@en-us&lt;/b&gt;&lt;p&gt;
Source: &lt;a href="https://assets.nexperia.com/documents/data-sheet/74LVC2G34.pdf"&gt; Datasheet &lt;/a&gt;</description>
<gates>
<gate name="G$1" symbol="74LVC2G34GW,125" x="0" y="0"/>
</gates>
<devices>
<device name="" package="SOP65P210X110-6N">
<connects>
<connect gate="G$1" pin="1A" pad="1"/>
<connect gate="G$1" pin="1Y" pad="6"/>
<connect gate="G$1" pin="2A" pad="3"/>
<connect gate="G$1" pin="2Y" pad="4"/>
<connect gate="G$1" pin="GND" pad="2"/>
<connect gate="G$1" pin="VCC" pad="5"/>
</connects>
<technologies>
<technology name="">
<attribute name="DESCRIPTION" value="74LVC2G34 - Dual buffer gate@en-us" constant="no"/>
<attribute name="HEIGHT" value="1.1mm" constant="no"/>
<attribute name="MANUFACTURER_NAME" value="Nexperia" constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="74LVC2G34GW,125" constant="no"/>
<attribute name="MOUSER_PART_NUMBER" value="771-LVC2G34GW125" constant="no"/>
<attribute name="MOUSER_PRICE-STOCK" value="https://www.mouser.co.uk/ProductDetail/Nexperia/74LVC2G34GW125?qs=me8TqzrmIYUOkg5J0C6mqA%3D%3D" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="USB_jacks">
<packages>
<package name="CONN5_ZX62D-B-5PA8_HIR">
<smd name="VBUS1" x="-1.3" y="2.675" dx="0.4064" dy="1.3462" layer="1"/>
<smd name="D-1" x="-0.65" y="2.675" dx="0.4064" dy="1.3462" layer="1"/>
<smd name="D+1" x="0" y="2.675" dx="0.4064" dy="1.3462" layer="1"/>
<smd name="ID1" x="0.65" y="2.675" dx="0.4064" dy="1.3462" layer="1"/>
<smd name="GND1" x="1.3" y="2.675" dx="0.4064" dy="1.3462" layer="1"/>
<pad name="MOUNT_1" x="-2.425" y="3" drill="0.85" diameter="1.25" rot="R90"/>
<pad name="MOUNT_2" x="2.425" y="3" drill="0.85" diameter="1.25" rot="R90"/>
<text x="-8.4972" y="3.0696" size="0.8128" layer="21" ratio="6" rot="SR0">&gt;Value</text>
<wire x1="-3.8" y1="-1.45" x2="3.8" y2="-1.45" width="0.1524" layer="51"/>
<wire x1="3.8" y1="-1.45" x2="3.8" y2="4.15" width="0.1524" layer="51"/>
<wire x1="3.8" y1="4.15" x2="-3.8" y2="4.15" width="0.1524" layer="51"/>
<wire x1="-3.8" y1="4.15" x2="-3.8" y2="-1.45" width="0.1524" layer="51"/>
<text x="-8.3566" y="1.5296" size="0.8128" layer="25" ratio="6" rot="SR0">&gt;Name</text>
<smd name="PAD_1" x="-1.15" y="0" dx="1.8" dy="1.9" layer="1"/>
<smd name="PAD_2" x="1.15" y="0" dx="1.8" dy="1.9" layer="1"/>
<pad name="MOUNT_4" x="-3.6" y="0" drill="1.5" diameter="1.9" shape="square"/>
<pad name="MOUNT_3" x="3.6" y="0" drill="1.5" diameter="1.9" shape="square"/>
</package>
</packages>
<symbols>
<symbol name="CONN5_ZX62D-B-5PA8">
<pin name="VBUS1" x="0" y="0" visible="pad" length="middle" direction="pas"/>
<pin name="D-1" x="0" y="-2.54" visible="pad" length="middle" direction="pas"/>
<pin name="D+1" x="0" y="-5.08" visible="pad" length="middle" direction="pas"/>
<pin name="ID1" x="0" y="-7.62" visible="pad" length="middle" direction="pas"/>
<pin name="GND1" x="0" y="-10.16" visible="pad" length="middle" direction="pas"/>
<wire x1="10.16" y1="0" x2="5.08" y2="0" width="0.1524" layer="94"/>
<wire x1="10.16" y1="-2.54" x2="5.08" y2="-2.54" width="0.1524" layer="94"/>
<wire x1="10.16" y1="-5.08" x2="5.08" y2="-5.08" width="0.1524" layer="94"/>
<wire x1="10.16" y1="-7.62" x2="5.08" y2="-7.62" width="0.1524" layer="94"/>
<wire x1="10.16" y1="-10.16" x2="5.08" y2="-10.16" width="0.1524" layer="94"/>
<wire x1="10.16" y1="0" x2="8.89" y2="0.8382" width="0.1524" layer="94"/>
<wire x1="10.16" y1="-2.54" x2="8.89" y2="-1.7018" width="0.1524" layer="94"/>
<wire x1="10.16" y1="-5.08" x2="8.89" y2="-4.2418" width="0.1524" layer="94"/>
<wire x1="10.16" y1="-7.62" x2="8.89" y2="-6.7818" width="0.1524" layer="94"/>
<wire x1="10.16" y1="-10.16" x2="8.89" y2="-9.3218" width="0.1524" layer="94"/>
<wire x1="10.16" y1="0" x2="8.89" y2="-0.8382" width="0.1524" layer="94"/>
<wire x1="10.16" y1="-2.54" x2="8.89" y2="-3.3782" width="0.1524" layer="94"/>
<wire x1="10.16" y1="-5.08" x2="8.89" y2="-5.9182" width="0.1524" layer="94"/>
<wire x1="10.16" y1="-7.62" x2="8.89" y2="-8.4582" width="0.1524" layer="94"/>
<wire x1="10.16" y1="-10.16" x2="8.89" y2="-10.9982" width="0.1524" layer="94"/>
<wire x1="5.08" y1="2.54" x2="5.08" y2="-12.7" width="0.1524" layer="94"/>
<wire x1="5.08" y1="-12.7" x2="12.7" y2="-12.7" width="0.1524" layer="94"/>
<wire x1="12.7" y1="-12.7" x2="12.7" y2="2.54" width="0.1524" layer="94"/>
<wire x1="12.7" y1="2.54" x2="5.08" y2="2.54" width="0.1524" layer="94"/>
<text x="4.1656" y="5.3086" size="2.0828" layer="95" ratio="6" rot="SR0">&gt;Name</text>
<pin name="CASE_GND" x="7.62" y="-17.78" visible="pad" length="middle" direction="pas" rot="R90"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="ZX62D-B-5PA8(30)" prefix="J">
<gates>
<gate name="A" symbol="CONN5_ZX62D-B-5PA8" x="0" y="0"/>
</gates>
<devices>
<device name="" package="CONN5_ZX62D-B-5PA8_HIR">
<connects>
<connect gate="A" pin="CASE_GND" pad="MOUNT_1 MOUNT_2 MOUNT_3 MOUNT_4 PAD_1 PAD_2" route="any"/>
<connect gate="A" pin="D+1" pad="D+1"/>
<connect gate="A" pin="D-1" pad="D-1"/>
<connect gate="A" pin="GND1" pad="GND1"/>
<connect gate="A" pin="ID1" pad="ID1"/>
<connect gate="A" pin="VBUS1" pad="VBUS1"/>
</connects>
<technologies>
<technology name="">
<attribute name="BUILT_BY" value="" constant="no"/>
<attribute name="COPYRIGHT" value="" constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="ZX62D-B-5PA8(30)" constant="no"/>
<attribute name="SOURCELIBRARY" value="Hirose_2018-12-15" constant="no"/>
<attribute name="VENDOR" value="Hirose" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="ATMEGA32U4">
<packages>
<package name="44ML_ATM">
<smd name="1" x="-5.7277" y="4" dx="0.4064" dy="1.3462" layer="1" rot="R270"/>
<smd name="2" x="-5.7277" y="3.2" dx="0.4064" dy="1.3462" layer="1" rot="R270"/>
<smd name="3" x="-5.7277" y="2.4" dx="0.4064" dy="1.3462" layer="1" rot="R270"/>
<smd name="4" x="-5.7277" y="1.6" dx="0.4064" dy="1.3462" layer="1" rot="R270"/>
<smd name="5" x="-5.7277" y="0.8" dx="0.4064" dy="1.3462" layer="1" rot="R270"/>
<smd name="6" x="-5.7277" y="0" dx="0.4064" dy="1.3462" layer="1" rot="R270"/>
<smd name="7" x="-5.7277" y="-0.8" dx="0.4064" dy="1.3462" layer="1" rot="R270"/>
<smd name="8" x="-5.7277" y="-1.6" dx="0.4064" dy="1.3462" layer="1" rot="R270"/>
<smd name="9" x="-5.7277" y="-2.4" dx="0.4064" dy="1.3462" layer="1" rot="R270"/>
<smd name="10" x="-5.7277" y="-3.2" dx="0.4064" dy="1.3462" layer="1" rot="R270"/>
<smd name="11" x="-5.7277" y="-4" dx="0.4064" dy="1.3462" layer="1" rot="R270"/>
<smd name="12" x="-4" y="-5.7277" dx="0.4064" dy="1.3462" layer="1" rot="R180"/>
<smd name="13" x="-3.2" y="-5.7277" dx="0.4064" dy="1.3462" layer="1" rot="R180"/>
<smd name="14" x="-2.4" y="-5.7277" dx="0.4064" dy="1.3462" layer="1" rot="R180"/>
<smd name="15" x="-1.6" y="-5.7277" dx="0.4064" dy="1.3462" layer="1" rot="R180"/>
<smd name="16" x="-0.8" y="-5.7277" dx="0.4064" dy="1.3462" layer="1" rot="R180"/>
<smd name="17" x="0" y="-5.7277" dx="0.4064" dy="1.3462" layer="1" rot="R180"/>
<smd name="18" x="0.8" y="-5.7277" dx="0.4064" dy="1.3462" layer="1" rot="R180"/>
<smd name="19" x="1.6" y="-5.7277" dx="0.4064" dy="1.3462" layer="1" rot="R180"/>
<smd name="20" x="2.4" y="-5.7277" dx="0.4064" dy="1.3462" layer="1" rot="R180"/>
<smd name="21" x="3.2" y="-5.7277" dx="0.4064" dy="1.3462" layer="1" rot="R180"/>
<smd name="22" x="4" y="-5.7277" dx="0.4064" dy="1.3462" layer="1" rot="R180"/>
<smd name="23" x="5.7277" y="-4" dx="0.4064" dy="1.3462" layer="1" rot="R270"/>
<smd name="24" x="5.7277" y="-3.2" dx="0.4064" dy="1.3462" layer="1" rot="R270"/>
<smd name="25" x="5.7277" y="-2.4" dx="0.4064" dy="1.3462" layer="1" rot="R270"/>
<smd name="26" x="5.7277" y="-1.6" dx="0.4064" dy="1.3462" layer="1" rot="R270"/>
<smd name="27" x="5.7277" y="-0.8" dx="0.4064" dy="1.3462" layer="1" rot="R270"/>
<smd name="28" x="5.7277" y="0" dx="0.4064" dy="1.3462" layer="1" rot="R270"/>
<smd name="29" x="5.7277" y="0.8" dx="0.4064" dy="1.3462" layer="1" rot="R270"/>
<smd name="30" x="5.7277" y="1.6" dx="0.4064" dy="1.3462" layer="1" rot="R270"/>
<smd name="31" x="5.7277" y="2.4" dx="0.4064" dy="1.3462" layer="1" rot="R270"/>
<smd name="32" x="5.7277" y="3.2" dx="0.4064" dy="1.3462" layer="1" rot="R270"/>
<smd name="33" x="5.7277" y="4" dx="0.4064" dy="1.3462" layer="1" rot="R270"/>
<smd name="34" x="4" y="5.7277" dx="0.4064" dy="1.3462" layer="1" rot="R180"/>
<smd name="35" x="3.2" y="5.7277" dx="0.4064" dy="1.3462" layer="1" rot="R180"/>
<smd name="36" x="2.4" y="5.7277" dx="0.4064" dy="1.3462" layer="1" rot="R180"/>
<smd name="37" x="1.6" y="5.7277" dx="0.4064" dy="1.3462" layer="1" rot="R180"/>
<smd name="38" x="0.8" y="5.7277" dx="0.4064" dy="1.3462" layer="1" rot="R180"/>
<smd name="39" x="0" y="5.7277" dx="0.4064" dy="1.3462" layer="1" rot="R180"/>
<smd name="40" x="-0.8" y="5.7277" dx="0.4064" dy="1.3462" layer="1" rot="R180"/>
<smd name="41" x="-1.6" y="5.7277" dx="0.4064" dy="1.3462" layer="1" rot="R180"/>
<smd name="42" x="-2.4" y="5.7277" dx="0.4064" dy="1.3462" layer="1" rot="R180"/>
<smd name="43" x="-3.2" y="5.7277" dx="0.4064" dy="1.3462" layer="1" rot="R180"/>
<smd name="44" x="-4" y="5.7277" dx="0.4064" dy="1.3462" layer="1" rot="R180"/>
<wire x1="-5.1816" y1="-5.1816" x2="-4.5466" y2="-5.1816" width="0.1524" layer="21"/>
<wire x1="5.1816" y1="-5.1816" x2="5.1816" y2="-4.5466" width="0.1524" layer="21"/>
<wire x1="5.1816" y1="5.1816" x2="4.5466" y2="5.1816" width="0.1524" layer="21"/>
<wire x1="-5.1816" y1="5.1816" x2="-5.1816" y2="4.5466" width="0.1524" layer="21"/>
<wire x1="-5.1816" y1="-4.5466" x2="-5.1816" y2="-5.1816" width="0.1524" layer="21"/>
<wire x1="4.5466" y1="-5.1816" x2="5.1816" y2="-5.1816" width="0.1524" layer="21"/>
<wire x1="5.1816" y1="4.5466" x2="5.1816" y2="5.1816" width="0.1524" layer="21"/>
<wire x1="-4.5466" y1="5.1816" x2="-5.1816" y2="5.1816" width="0.1524" layer="21"/>
<polygon width="0.0254" layer="21">
<vertex x="-6.9088" y="-3.0095"/>
<vertex x="-6.9088" y="-3.3905"/>
<vertex x="-6.6548" y="-3.3905"/>
<vertex x="-6.6548" y="-3.0095"/>
</polygon>
<polygon width="0.0254" layer="21">
<vertex x="2.2095" y="-6.6548"/>
<vertex x="2.2095" y="-6.9088"/>
<vertex x="2.5905" y="-6.9088"/>
<vertex x="2.5905" y="-6.6548"/>
</polygon>
<polygon width="0.0254" layer="21">
<vertex x="6.9088" y="1.7905"/>
<vertex x="6.9088" y="1.4095"/>
<vertex x="6.6548" y="1.4095"/>
<vertex x="6.6548" y="1.7905"/>
</polygon>
<polygon width="0.0254" layer="21">
<vertex x="-0.9905" y="6.6548"/>
<vertex x="-0.9905" y="6.9088"/>
<vertex x="-0.6095" y="6.9088"/>
<vertex x="-0.6095" y="6.6548"/>
</polygon>
<text x="-7.62" y="4.1402" size="1.27" layer="21" ratio="6" rot="SR0">*</text>
<text x="-1.7272" y="-0.635" size="1.27" layer="21" ratio="6" rot="SR0">&gt;Value</text>
<wire x1="3.81" y1="5.0546" x2="4.1656" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="4.1656" y1="5.0546" x2="4.1656" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="4.1656" y1="6.0452" x2="3.81" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="3.81" y1="6.0452" x2="3.81" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="3.0226" y1="5.0546" x2="3.3782" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="3.3782" y1="5.0546" x2="3.3782" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="3.3782" y1="6.0452" x2="3.0226" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="3.0226" y1="6.0452" x2="3.0226" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="2.2098" y1="5.0546" x2="2.5654" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="2.5654" y1="5.0546" x2="2.5654" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="2.5654" y1="6.0452" x2="2.2098" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="2.2098" y1="6.0452" x2="2.2098" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="1.4224" y1="5.0546" x2="1.778" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="1.778" y1="5.0546" x2="1.778" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="1.778" y1="6.0452" x2="1.4224" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="1.4224" y1="6.0452" x2="1.4224" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="0.6096" y1="5.0546" x2="0.9652" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="0.9652" y1="5.0546" x2="0.9652" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="0.9652" y1="6.0452" x2="0.6096" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="0.6096" y1="6.0452" x2="0.6096" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="-0.1778" y1="5.0546" x2="0.1778" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="0.1778" y1="5.0546" x2="0.1778" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="0.1778" y1="6.0452" x2="-0.1778" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="-0.1778" y1="6.0452" x2="-0.1778" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="-0.9652" y1="5.0546" x2="-0.6096" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="-0.6096" y1="5.0546" x2="-0.6096" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="-0.6096" y1="6.0452" x2="-0.9652" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="-0.9652" y1="6.0452" x2="-0.9652" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="-1.778" y1="5.0546" x2="-1.4224" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="-1.4224" y1="5.0546" x2="-1.4224" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="-1.4224" y1="6.0452" x2="-1.778" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="-1.778" y1="6.0452" x2="-1.778" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="-2.5654" y1="5.0546" x2="-2.2098" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="-2.2098" y1="5.0546" x2="-2.2098" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="-2.2098" y1="6.0452" x2="-2.5654" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="-2.5654" y1="6.0452" x2="-2.5654" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="-3.3782" y1="5.0546" x2="-3.0226" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="-3.0226" y1="5.0546" x2="-3.0226" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="-3.0226" y1="6.0452" x2="-3.3782" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="-3.3782" y1="6.0452" x2="-3.3782" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="-4.1656" y1="5.0546" x2="-3.81" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="-3.81" y1="5.0546" x2="-3.81" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="-3.81" y1="6.0452" x2="-4.1656" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="-4.1656" y1="6.0452" x2="-4.1656" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="3.81" x2="-5.0546" y2="4.1656" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="4.1656" x2="-6.0452" y2="4.1656" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="4.1656" x2="-6.0452" y2="3.81" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="3.81" x2="-5.0546" y2="3.81" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="3.0226" x2="-5.0546" y2="3.3782" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="3.3782" x2="-6.0452" y2="3.3782" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="3.3782" x2="-6.0452" y2="3.0226" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="3.0226" x2="-5.0546" y2="3.0226" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="2.2098" x2="-5.0546" y2="2.5654" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="2.5654" x2="-6.0452" y2="2.5654" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="2.5654" x2="-6.0452" y2="2.2098" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="2.2098" x2="-5.0546" y2="2.2098" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="1.4224" x2="-5.0546" y2="1.778" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="1.778" x2="-6.0452" y2="1.778" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="1.778" x2="-6.0452" y2="1.4224" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="1.4224" x2="-5.0546" y2="1.4224" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="0.6096" x2="-5.0546" y2="0.9652" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="0.9652" x2="-6.0452" y2="0.9652" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="0.9652" x2="-6.0452" y2="0.6096" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="0.6096" x2="-5.0546" y2="0.6096" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="-0.1778" x2="-5.0546" y2="0.1778" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="0.1778" x2="-6.0452" y2="0.1778" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="0.1778" x2="-6.0452" y2="-0.1778" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="-0.1778" x2="-5.0546" y2="-0.1778" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="-0.9652" x2="-5.0546" y2="-0.6096" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="-0.6096" x2="-6.0452" y2="-0.6096" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="-0.6096" x2="-6.0452" y2="-0.9652" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="-0.9652" x2="-5.0546" y2="-0.9652" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="-1.778" x2="-5.0546" y2="-1.4224" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="-1.4224" x2="-6.0452" y2="-1.4224" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="-1.4224" x2="-6.0452" y2="-1.778" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="-1.778" x2="-5.0546" y2="-1.778" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="-2.5654" x2="-5.0546" y2="-2.2098" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="-2.2098" x2="-6.0452" y2="-2.2098" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="-2.2098" x2="-6.0452" y2="-2.5654" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="-2.5654" x2="-5.0546" y2="-2.5654" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="-3.3782" x2="-5.0546" y2="-3.0226" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="-3.0226" x2="-6.0452" y2="-3.0226" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="-3.0226" x2="-6.0452" y2="-3.3782" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="-3.3782" x2="-5.0546" y2="-3.3782" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="-4.1656" x2="-5.0546" y2="-3.81" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="-3.81" x2="-6.0452" y2="-3.81" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="-3.81" x2="-6.0452" y2="-4.1656" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="-4.1656" x2="-5.0546" y2="-4.1656" width="0.1524" layer="51"/>
<wire x1="-3.81" y1="-5.0546" x2="-4.1656" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="-4.1656" y1="-5.0546" x2="-4.1656" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="-4.1656" y1="-6.0452" x2="-3.81" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="-3.81" y1="-6.0452" x2="-3.81" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="-3.0226" y1="-5.0546" x2="-3.3782" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="-3.3782" y1="-5.0546" x2="-3.3782" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="-3.3782" y1="-6.0452" x2="-3.0226" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="-3.0226" y1="-6.0452" x2="-3.0226" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="-2.2098" y1="-5.0546" x2="-2.5654" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="-2.5654" y1="-5.0546" x2="-2.5654" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="-2.5654" y1="-6.0452" x2="-2.2098" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="-2.2098" y1="-6.0452" x2="-2.2098" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="-1.4224" y1="-5.0546" x2="-1.778" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="-1.778" y1="-5.0546" x2="-1.778" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="-1.778" y1="-6.0452" x2="-1.4224" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="-1.4224" y1="-6.0452" x2="-1.4224" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="-0.6096" y1="-5.0546" x2="-0.9652" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="-0.9652" y1="-5.0546" x2="-0.9652" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="-0.9652" y1="-6.0452" x2="-0.6096" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="-0.6096" y1="-6.0452" x2="-0.6096" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="0.1778" y1="-5.0546" x2="-0.1778" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="-0.1778" y1="-5.0546" x2="-0.1778" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="-0.1778" y1="-6.0452" x2="0.1778" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="0.1778" y1="-6.0452" x2="0.1778" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="0.9652" y1="-5.0546" x2="0.6096" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="0.6096" y1="-5.0546" x2="0.6096" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="0.6096" y1="-6.0452" x2="0.9652" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="0.9652" y1="-6.0452" x2="0.9652" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="1.778" y1="-5.0546" x2="1.4224" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="1.4224" y1="-5.0546" x2="1.4224" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="1.4224" y1="-6.0452" x2="1.778" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="1.778" y1="-6.0452" x2="1.778" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="2.5654" y1="-5.0546" x2="2.2098" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="2.2098" y1="-5.0546" x2="2.2098" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="2.2098" y1="-6.0452" x2="2.5654" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="2.5654" y1="-6.0452" x2="2.5654" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="3.3782" y1="-5.0546" x2="3.0226" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="3.0226" y1="-5.0546" x2="3.0226" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="3.0226" y1="-6.0452" x2="3.3782" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="3.3782" y1="-6.0452" x2="3.3782" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="4.1656" y1="-5.0546" x2="3.81" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="3.81" y1="-5.0546" x2="3.81" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="3.81" y1="-6.0452" x2="4.1656" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="4.1656" y1="-6.0452" x2="4.1656" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="-3.81" x2="5.0546" y2="-4.1656" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="-4.1656" x2="6.0452" y2="-4.1656" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="-4.1656" x2="6.0452" y2="-3.81" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="-3.81" x2="5.0546" y2="-3.81" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="-3.0226" x2="5.0546" y2="-3.3782" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="-3.3782" x2="6.0452" y2="-3.3782" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="-3.3782" x2="6.0452" y2="-3.0226" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="-3.0226" x2="5.0546" y2="-3.0226" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="-2.2098" x2="5.0546" y2="-2.5654" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="-2.5654" x2="6.0452" y2="-2.5654" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="-2.5654" x2="6.0452" y2="-2.2098" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="-2.2098" x2="5.0546" y2="-2.2098" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="-1.4224" x2="5.0546" y2="-1.778" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="-1.778" x2="6.0452" y2="-1.778" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="-1.778" x2="6.0452" y2="-1.4224" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="-1.4224" x2="5.0546" y2="-1.4224" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="-0.6096" x2="5.0546" y2="-0.9652" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="-0.9652" x2="6.0452" y2="-0.9652" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="-0.9652" x2="6.0452" y2="-0.6096" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="-0.6096" x2="5.0546" y2="-0.6096" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="0.1778" x2="5.0546" y2="-0.1778" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="-0.1778" x2="6.0452" y2="-0.1778" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="-0.1778" x2="6.0452" y2="0.1778" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="0.1778" x2="5.0546" y2="0.1778" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="0.9652" x2="5.0546" y2="0.6096" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="0.6096" x2="6.0452" y2="0.6096" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="0.6096" x2="6.0452" y2="0.9652" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="0.9652" x2="5.0546" y2="0.9652" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="1.778" x2="5.0546" y2="1.4224" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="1.4224" x2="6.0452" y2="1.4224" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="1.4224" x2="6.0452" y2="1.778" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="1.778" x2="5.0546" y2="1.778" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="2.5654" x2="5.0546" y2="2.2098" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="2.2098" x2="6.0452" y2="2.2098" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="2.2098" x2="6.0452" y2="2.5654" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="2.5654" x2="5.0546" y2="2.5654" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="3.3782" x2="5.0546" y2="3.0226" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="3.0226" x2="6.0452" y2="3.0226" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="3.0226" x2="6.0452" y2="3.3782" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="3.3782" x2="5.0546" y2="3.3782" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="4.1656" x2="5.0546" y2="3.81" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="3.81" x2="6.0452" y2="3.81" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="3.81" x2="6.0452" y2="4.1656" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="4.1656" x2="5.0546" y2="4.1656" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="3.7846" x2="-3.7846" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="-5.0546" x2="5.0546" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="-5.0546" x2="5.0546" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="5.0546" x2="-5.0546" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="5.0546" x2="-5.0546" y2="-5.0546" width="0.1524" layer="51"/>
<text x="-5.2578" y="3.7592" size="1.27" layer="51" ratio="6" rot="SR0">*</text>
<text x="-3.2766" y="-0.635" size="1.27" layer="25" ratio="6" rot="SR0">&gt;Name</text>
</package>
<package name="44ML_ATM-M">
<smd name="1" x="-5.8293" y="4" dx="0.4064" dy="1.5494" layer="1" rot="R270"/>
<smd name="2" x="-5.8293" y="3.2" dx="0.4064" dy="1.5494" layer="1" rot="R270"/>
<smd name="3" x="-5.8293" y="2.4" dx="0.4064" dy="1.5494" layer="1" rot="R270"/>
<smd name="4" x="-5.8293" y="1.6" dx="0.4064" dy="1.5494" layer="1" rot="R270"/>
<smd name="5" x="-5.8293" y="0.8" dx="0.4064" dy="1.5494" layer="1" rot="R270"/>
<smd name="6" x="-5.8293" y="0" dx="0.4064" dy="1.5494" layer="1" rot="R270"/>
<smd name="7" x="-5.8293" y="-0.8" dx="0.4064" dy="1.5494" layer="1" rot="R270"/>
<smd name="8" x="-5.8293" y="-1.6" dx="0.4064" dy="1.5494" layer="1" rot="R270"/>
<smd name="9" x="-5.8293" y="-2.4" dx="0.4064" dy="1.5494" layer="1" rot="R270"/>
<smd name="10" x="-5.8293" y="-3.2" dx="0.4064" dy="1.5494" layer="1" rot="R270"/>
<smd name="11" x="-5.8293" y="-4" dx="0.4064" dy="1.5494" layer="1" rot="R270"/>
<smd name="12" x="-4" y="-5.8293" dx="0.4064" dy="1.5494" layer="1" rot="R180"/>
<smd name="13" x="-3.2" y="-5.8293" dx="0.4064" dy="1.5494" layer="1" rot="R180"/>
<smd name="14" x="-2.4" y="-5.8293" dx="0.4064" dy="1.5494" layer="1" rot="R180"/>
<smd name="15" x="-1.6" y="-5.8293" dx="0.4064" dy="1.5494" layer="1" rot="R180"/>
<smd name="16" x="-0.8" y="-5.8293" dx="0.4064" dy="1.5494" layer="1" rot="R180"/>
<smd name="17" x="0" y="-5.8293" dx="0.4064" dy="1.5494" layer="1" rot="R180"/>
<smd name="18" x="0.8" y="-5.8293" dx="0.4064" dy="1.5494" layer="1" rot="R180"/>
<smd name="19" x="1.6" y="-5.8293" dx="0.4064" dy="1.5494" layer="1" rot="R180"/>
<smd name="20" x="2.4" y="-5.8293" dx="0.4064" dy="1.5494" layer="1" rot="R180"/>
<smd name="21" x="3.2" y="-5.8293" dx="0.4064" dy="1.5494" layer="1" rot="R180"/>
<smd name="22" x="4" y="-5.8293" dx="0.4064" dy="1.5494" layer="1" rot="R180"/>
<smd name="23" x="5.8293" y="-4" dx="0.4064" dy="1.5494" layer="1" rot="R270"/>
<smd name="24" x="5.8293" y="-3.2" dx="0.4064" dy="1.5494" layer="1" rot="R270"/>
<smd name="25" x="5.8293" y="-2.4" dx="0.4064" dy="1.5494" layer="1" rot="R270"/>
<smd name="26" x="5.8293" y="-1.6" dx="0.4064" dy="1.5494" layer="1" rot="R270"/>
<smd name="27" x="5.8293" y="-0.8" dx="0.4064" dy="1.5494" layer="1" rot="R270"/>
<smd name="28" x="5.8293" y="0" dx="0.4064" dy="1.5494" layer="1" rot="R270"/>
<smd name="29" x="5.8293" y="0.8" dx="0.4064" dy="1.5494" layer="1" rot="R270"/>
<smd name="30" x="5.8293" y="1.6" dx="0.4064" dy="1.5494" layer="1" rot="R270"/>
<smd name="31" x="5.8293" y="2.4" dx="0.4064" dy="1.5494" layer="1" rot="R270"/>
<smd name="32" x="5.8293" y="3.2" dx="0.4064" dy="1.5494" layer="1" rot="R270"/>
<smd name="33" x="5.8293" y="4" dx="0.4064" dy="1.5494" layer="1" rot="R270"/>
<smd name="34" x="4" y="5.8293" dx="0.4064" dy="1.5494" layer="1" rot="R180"/>
<smd name="35" x="3.2" y="5.8293" dx="0.4064" dy="1.5494" layer="1" rot="R180"/>
<smd name="36" x="2.4" y="5.8293" dx="0.4064" dy="1.5494" layer="1" rot="R180"/>
<smd name="37" x="1.6" y="5.8293" dx="0.4064" dy="1.5494" layer="1" rot="R180"/>
<smd name="38" x="0.8" y="5.8293" dx="0.4064" dy="1.5494" layer="1" rot="R180"/>
<smd name="39" x="0" y="5.8293" dx="0.4064" dy="1.5494" layer="1" rot="R180"/>
<smd name="40" x="-0.8" y="5.8293" dx="0.4064" dy="1.5494" layer="1" rot="R180"/>
<smd name="41" x="-1.6" y="5.8293" dx="0.4064" dy="1.5494" layer="1" rot="R180"/>
<smd name="42" x="-2.4" y="5.8293" dx="0.4064" dy="1.5494" layer="1" rot="R180"/>
<smd name="43" x="-3.2" y="5.8293" dx="0.4064" dy="1.5494" layer="1" rot="R180"/>
<smd name="44" x="-4" y="5.8293" dx="0.4064" dy="1.5494" layer="1" rot="R180"/>
<wire x1="-5.1816" y1="-5.1816" x2="-4.5466" y2="-5.1816" width="0.1524" layer="21"/>
<wire x1="5.1816" y1="-5.1816" x2="5.1816" y2="-4.5466" width="0.1524" layer="21"/>
<wire x1="5.1816" y1="5.1816" x2="4.5466" y2="5.1816" width="0.1524" layer="21"/>
<wire x1="-5.1816" y1="5.1816" x2="-5.1816" y2="4.5466" width="0.1524" layer="21"/>
<wire x1="-5.1816" y1="-4.5466" x2="-5.1816" y2="-5.1816" width="0.1524" layer="21"/>
<wire x1="4.5466" y1="-5.1816" x2="5.1816" y2="-5.1816" width="0.1524" layer="21"/>
<wire x1="5.1816" y1="4.5466" x2="5.1816" y2="5.1816" width="0.1524" layer="21"/>
<wire x1="-4.5466" y1="5.1816" x2="-5.1816" y2="5.1816" width="0.1524" layer="21"/>
<polygon width="0.0254" layer="21">
<vertex x="-7.112" y="-3.0095"/>
<vertex x="-7.112" y="-3.3905"/>
<vertex x="-6.858" y="-3.3905"/>
<vertex x="-6.858" y="-3.0095"/>
</polygon>
<polygon width="0.0254" layer="21">
<vertex x="2.2095" y="-6.858"/>
<vertex x="2.2095" y="-7.112"/>
<vertex x="2.5905" y="-7.112"/>
<vertex x="2.5905" y="-6.858"/>
</polygon>
<polygon width="0.0254" layer="21">
<vertex x="7.112" y="1.7905"/>
<vertex x="7.112" y="1.4095"/>
<vertex x="6.858" y="1.4095"/>
<vertex x="6.858" y="1.7905"/>
</polygon>
<polygon width="0.0254" layer="21">
<vertex x="-0.9905" y="6.858"/>
<vertex x="-0.9905" y="7.112"/>
<vertex x="-0.6095" y="7.112"/>
<vertex x="-0.6095" y="6.858"/>
</polygon>
<text x="-7.8232" y="4.1402" size="1.27" layer="21" ratio="6" rot="SR0">*</text>
<text x="-1.7272" y="-0.635" size="1.27" layer="21" ratio="6" rot="SR0">&gt;Value</text>
<wire x1="3.81" y1="5.0546" x2="4.1656" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="4.1656" y1="5.0546" x2="4.1656" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="4.1656" y1="6.0452" x2="3.81" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="3.81" y1="6.0452" x2="3.81" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="3.0226" y1="5.0546" x2="3.3782" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="3.3782" y1="5.0546" x2="3.3782" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="3.3782" y1="6.0452" x2="3.0226" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="3.0226" y1="6.0452" x2="3.0226" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="2.2098" y1="5.0546" x2="2.5654" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="2.5654" y1="5.0546" x2="2.5654" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="2.5654" y1="6.0452" x2="2.2098" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="2.2098" y1="6.0452" x2="2.2098" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="1.4224" y1="5.0546" x2="1.778" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="1.778" y1="5.0546" x2="1.778" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="1.778" y1="6.0452" x2="1.4224" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="1.4224" y1="6.0452" x2="1.4224" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="0.6096" y1="5.0546" x2="0.9652" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="0.9652" y1="5.0546" x2="0.9652" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="0.9652" y1="6.0452" x2="0.6096" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="0.6096" y1="6.0452" x2="0.6096" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="-0.1778" y1="5.0546" x2="0.1778" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="0.1778" y1="5.0546" x2="0.1778" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="0.1778" y1="6.0452" x2="-0.1778" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="-0.1778" y1="6.0452" x2="-0.1778" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="-0.9652" y1="5.0546" x2="-0.6096" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="-0.6096" y1="5.0546" x2="-0.6096" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="-0.6096" y1="6.0452" x2="-0.9652" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="-0.9652" y1="6.0452" x2="-0.9652" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="-1.778" y1="5.0546" x2="-1.4224" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="-1.4224" y1="5.0546" x2="-1.4224" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="-1.4224" y1="6.0452" x2="-1.778" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="-1.778" y1="6.0452" x2="-1.778" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="-2.5654" y1="5.0546" x2="-2.2098" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="-2.2098" y1="5.0546" x2="-2.2098" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="-2.2098" y1="6.0452" x2="-2.5654" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="-2.5654" y1="6.0452" x2="-2.5654" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="-3.3782" y1="5.0546" x2="-3.0226" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="-3.0226" y1="5.0546" x2="-3.0226" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="-3.0226" y1="6.0452" x2="-3.3782" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="-3.3782" y1="6.0452" x2="-3.3782" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="-4.1656" y1="5.0546" x2="-3.81" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="-3.81" y1="5.0546" x2="-3.81" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="-3.81" y1="6.0452" x2="-4.1656" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="-4.1656" y1="6.0452" x2="-4.1656" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="3.81" x2="-5.0546" y2="4.1656" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="4.1656" x2="-6.0452" y2="4.1656" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="4.1656" x2="-6.0452" y2="3.81" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="3.81" x2="-5.0546" y2="3.81" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="3.0226" x2="-5.0546" y2="3.3782" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="3.3782" x2="-6.0452" y2="3.3782" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="3.3782" x2="-6.0452" y2="3.0226" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="3.0226" x2="-5.0546" y2="3.0226" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="2.2098" x2="-5.0546" y2="2.5654" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="2.5654" x2="-6.0452" y2="2.5654" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="2.5654" x2="-6.0452" y2="2.2098" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="2.2098" x2="-5.0546" y2="2.2098" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="1.4224" x2="-5.0546" y2="1.778" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="1.778" x2="-6.0452" y2="1.778" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="1.778" x2="-6.0452" y2="1.4224" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="1.4224" x2="-5.0546" y2="1.4224" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="0.6096" x2="-5.0546" y2="0.9652" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="0.9652" x2="-6.0452" y2="0.9652" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="0.9652" x2="-6.0452" y2="0.6096" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="0.6096" x2="-5.0546" y2="0.6096" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="-0.1778" x2="-5.0546" y2="0.1778" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="0.1778" x2="-6.0452" y2="0.1778" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="0.1778" x2="-6.0452" y2="-0.1778" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="-0.1778" x2="-5.0546" y2="-0.1778" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="-0.9652" x2="-5.0546" y2="-0.6096" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="-0.6096" x2="-6.0452" y2="-0.6096" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="-0.6096" x2="-6.0452" y2="-0.9652" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="-0.9652" x2="-5.0546" y2="-0.9652" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="-1.778" x2="-5.0546" y2="-1.4224" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="-1.4224" x2="-6.0452" y2="-1.4224" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="-1.4224" x2="-6.0452" y2="-1.778" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="-1.778" x2="-5.0546" y2="-1.778" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="-2.5654" x2="-5.0546" y2="-2.2098" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="-2.2098" x2="-6.0452" y2="-2.2098" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="-2.2098" x2="-6.0452" y2="-2.5654" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="-2.5654" x2="-5.0546" y2="-2.5654" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="-3.3782" x2="-5.0546" y2="-3.0226" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="-3.0226" x2="-6.0452" y2="-3.0226" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="-3.0226" x2="-6.0452" y2="-3.3782" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="-3.3782" x2="-5.0546" y2="-3.3782" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="-4.1656" x2="-5.0546" y2="-3.81" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="-3.81" x2="-6.0452" y2="-3.81" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="-3.81" x2="-6.0452" y2="-4.1656" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="-4.1656" x2="-5.0546" y2="-4.1656" width="0.1524" layer="51"/>
<wire x1="-3.81" y1="-5.0546" x2="-4.1656" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="-4.1656" y1="-5.0546" x2="-4.1656" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="-4.1656" y1="-6.0452" x2="-3.81" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="-3.81" y1="-6.0452" x2="-3.81" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="-3.0226" y1="-5.0546" x2="-3.3782" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="-3.3782" y1="-5.0546" x2="-3.3782" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="-3.3782" y1="-6.0452" x2="-3.0226" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="-3.0226" y1="-6.0452" x2="-3.0226" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="-2.2098" y1="-5.0546" x2="-2.5654" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="-2.5654" y1="-5.0546" x2="-2.5654" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="-2.5654" y1="-6.0452" x2="-2.2098" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="-2.2098" y1="-6.0452" x2="-2.2098" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="-1.4224" y1="-5.0546" x2="-1.778" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="-1.778" y1="-5.0546" x2="-1.778" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="-1.778" y1="-6.0452" x2="-1.4224" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="-1.4224" y1="-6.0452" x2="-1.4224" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="-0.6096" y1="-5.0546" x2="-0.9652" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="-0.9652" y1="-5.0546" x2="-0.9652" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="-0.9652" y1="-6.0452" x2="-0.6096" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="-0.6096" y1="-6.0452" x2="-0.6096" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="0.1778" y1="-5.0546" x2="-0.1778" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="-0.1778" y1="-5.0546" x2="-0.1778" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="-0.1778" y1="-6.0452" x2="0.1778" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="0.1778" y1="-6.0452" x2="0.1778" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="0.9652" y1="-5.0546" x2="0.6096" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="0.6096" y1="-5.0546" x2="0.6096" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="0.6096" y1="-6.0452" x2="0.9652" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="0.9652" y1="-6.0452" x2="0.9652" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="1.778" y1="-5.0546" x2="1.4224" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="1.4224" y1="-5.0546" x2="1.4224" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="1.4224" y1="-6.0452" x2="1.778" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="1.778" y1="-6.0452" x2="1.778" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="2.5654" y1="-5.0546" x2="2.2098" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="2.2098" y1="-5.0546" x2="2.2098" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="2.2098" y1="-6.0452" x2="2.5654" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="2.5654" y1="-6.0452" x2="2.5654" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="3.3782" y1="-5.0546" x2="3.0226" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="3.0226" y1="-5.0546" x2="3.0226" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="3.0226" y1="-6.0452" x2="3.3782" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="3.3782" y1="-6.0452" x2="3.3782" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="4.1656" y1="-5.0546" x2="3.81" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="3.81" y1="-5.0546" x2="3.81" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="3.81" y1="-6.0452" x2="4.1656" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="4.1656" y1="-6.0452" x2="4.1656" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="-3.81" x2="5.0546" y2="-4.1656" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="-4.1656" x2="6.0452" y2="-4.1656" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="-4.1656" x2="6.0452" y2="-3.81" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="-3.81" x2="5.0546" y2="-3.81" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="-3.0226" x2="5.0546" y2="-3.3782" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="-3.3782" x2="6.0452" y2="-3.3782" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="-3.3782" x2="6.0452" y2="-3.0226" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="-3.0226" x2="5.0546" y2="-3.0226" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="-2.2098" x2="5.0546" y2="-2.5654" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="-2.5654" x2="6.0452" y2="-2.5654" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="-2.5654" x2="6.0452" y2="-2.2098" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="-2.2098" x2="5.0546" y2="-2.2098" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="-1.4224" x2="5.0546" y2="-1.778" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="-1.778" x2="6.0452" y2="-1.778" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="-1.778" x2="6.0452" y2="-1.4224" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="-1.4224" x2="5.0546" y2="-1.4224" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="-0.6096" x2="5.0546" y2="-0.9652" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="-0.9652" x2="6.0452" y2="-0.9652" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="-0.9652" x2="6.0452" y2="-0.6096" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="-0.6096" x2="5.0546" y2="-0.6096" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="0.1778" x2="5.0546" y2="-0.1778" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="-0.1778" x2="6.0452" y2="-0.1778" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="-0.1778" x2="6.0452" y2="0.1778" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="0.1778" x2="5.0546" y2="0.1778" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="0.9652" x2="5.0546" y2="0.6096" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="0.6096" x2="6.0452" y2="0.6096" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="0.6096" x2="6.0452" y2="0.9652" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="0.9652" x2="5.0546" y2="0.9652" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="1.778" x2="5.0546" y2="1.4224" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="1.4224" x2="6.0452" y2="1.4224" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="1.4224" x2="6.0452" y2="1.778" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="1.778" x2="5.0546" y2="1.778" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="2.5654" x2="5.0546" y2="2.2098" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="2.2098" x2="6.0452" y2="2.2098" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="2.2098" x2="6.0452" y2="2.5654" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="2.5654" x2="5.0546" y2="2.5654" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="3.3782" x2="5.0546" y2="3.0226" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="3.0226" x2="6.0452" y2="3.0226" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="3.0226" x2="6.0452" y2="3.3782" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="3.3782" x2="5.0546" y2="3.3782" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="4.1656" x2="5.0546" y2="3.81" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="3.81" x2="6.0452" y2="3.81" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="3.81" x2="6.0452" y2="4.1656" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="4.1656" x2="5.0546" y2="4.1656" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="3.7846" x2="-3.7846" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="-5.0546" x2="5.0546" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="-5.0546" x2="5.0546" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="5.0546" x2="-5.0546" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="5.0546" x2="-5.0546" y2="-5.0546" width="0.1524" layer="51"/>
<text x="-5.2578" y="3.7592" size="1.27" layer="51" ratio="6" rot="SR0">*</text>
<text x="-3.2766" y="-0.635" size="1.27" layer="25" ratio="6" rot="SR0">&gt;Name</text>
</package>
<package name="44ML_ATM-L">
<smd name="1" x="-5.6261" y="4" dx="0.3556" dy="1.143" layer="1" rot="R270"/>
<smd name="2" x="-5.6261" y="3.2" dx="0.3556" dy="1.143" layer="1" rot="R270"/>
<smd name="3" x="-5.6261" y="2.4" dx="0.3556" dy="1.143" layer="1" rot="R270"/>
<smd name="4" x="-5.6261" y="1.6" dx="0.3556" dy="1.143" layer="1" rot="R270"/>
<smd name="5" x="-5.6261" y="0.8" dx="0.3556" dy="1.143" layer="1" rot="R270"/>
<smd name="6" x="-5.6261" y="0" dx="0.3556" dy="1.143" layer="1" rot="R270"/>
<smd name="7" x="-5.6261" y="-0.8" dx="0.3556" dy="1.143" layer="1" rot="R270"/>
<smd name="8" x="-5.6261" y="-1.6" dx="0.3556" dy="1.143" layer="1" rot="R270"/>
<smd name="9" x="-5.6261" y="-2.4" dx="0.3556" dy="1.143" layer="1" rot="R270"/>
<smd name="10" x="-5.6261" y="-3.2" dx="0.3556" dy="1.143" layer="1" rot="R270"/>
<smd name="11" x="-5.6261" y="-4" dx="0.3556" dy="1.143" layer="1" rot="R270"/>
<smd name="12" x="-4" y="-5.6261" dx="0.3556" dy="1.143" layer="1" rot="R180"/>
<smd name="13" x="-3.2" y="-5.6261" dx="0.3556" dy="1.143" layer="1" rot="R180"/>
<smd name="14" x="-2.4" y="-5.6261" dx="0.3556" dy="1.143" layer="1" rot="R180"/>
<smd name="15" x="-1.6" y="-5.6261" dx="0.3556" dy="1.143" layer="1" rot="R180"/>
<smd name="16" x="-0.8" y="-5.6261" dx="0.3556" dy="1.143" layer="1" rot="R180"/>
<smd name="17" x="0" y="-5.6261" dx="0.3556" dy="1.143" layer="1" rot="R180"/>
<smd name="18" x="0.8" y="-5.6261" dx="0.3556" dy="1.143" layer="1" rot="R180"/>
<smd name="19" x="1.6" y="-5.6261" dx="0.3556" dy="1.143" layer="1" rot="R180"/>
<smd name="20" x="2.4" y="-5.6261" dx="0.3556" dy="1.143" layer="1" rot="R180"/>
<smd name="21" x="3.2" y="-5.6261" dx="0.3556" dy="1.143" layer="1" rot="R180"/>
<smd name="22" x="4" y="-5.6261" dx="0.3556" dy="1.143" layer="1" rot="R180"/>
<smd name="23" x="5.6261" y="-4" dx="0.3556" dy="1.143" layer="1" rot="R270"/>
<smd name="24" x="5.6261" y="-3.2" dx="0.3556" dy="1.143" layer="1" rot="R270"/>
<smd name="25" x="5.6261" y="-2.4" dx="0.3556" dy="1.143" layer="1" rot="R270"/>
<smd name="26" x="5.6261" y="-1.6" dx="0.3556" dy="1.143" layer="1" rot="R270"/>
<smd name="27" x="5.6261" y="-0.8" dx="0.3556" dy="1.143" layer="1" rot="R270"/>
<smd name="28" x="5.6261" y="0" dx="0.3556" dy="1.143" layer="1" rot="R270"/>
<smd name="29" x="5.6261" y="0.8" dx="0.3556" dy="1.143" layer="1" rot="R270"/>
<smd name="30" x="5.6261" y="1.6" dx="0.3556" dy="1.143" layer="1" rot="R270"/>
<smd name="31" x="5.6261" y="2.4" dx="0.3556" dy="1.143" layer="1" rot="R270"/>
<smd name="32" x="5.6261" y="3.2" dx="0.3556" dy="1.143" layer="1" rot="R270"/>
<smd name="33" x="5.6261" y="4" dx="0.3556" dy="1.143" layer="1" rot="R270"/>
<smd name="34" x="4" y="5.6261" dx="0.3556" dy="1.143" layer="1" rot="R180"/>
<smd name="35" x="3.2" y="5.6261" dx="0.3556" dy="1.143" layer="1" rot="R180"/>
<smd name="36" x="2.4" y="5.6261" dx="0.3556" dy="1.143" layer="1" rot="R180"/>
<smd name="37" x="1.6" y="5.6261" dx="0.3556" dy="1.143" layer="1" rot="R180"/>
<smd name="38" x="0.8" y="5.6261" dx="0.3556" dy="1.143" layer="1" rot="R180"/>
<smd name="39" x="0" y="5.6261" dx="0.3556" dy="1.143" layer="1" rot="R180"/>
<smd name="40" x="-0.8" y="5.6261" dx="0.3556" dy="1.143" layer="1" rot="R180"/>
<smd name="41" x="-1.6" y="5.6261" dx="0.3556" dy="1.143" layer="1" rot="R180"/>
<smd name="42" x="-2.4" y="5.6261" dx="0.3556" dy="1.143" layer="1" rot="R180"/>
<smd name="43" x="-3.2" y="5.6261" dx="0.3556" dy="1.143" layer="1" rot="R180"/>
<smd name="44" x="-4" y="5.6261" dx="0.3556" dy="1.143" layer="1" rot="R180"/>
<wire x1="-5.1816" y1="-5.1816" x2="-4.5212" y2="-5.1816" width="0.1524" layer="21"/>
<wire x1="5.1816" y1="-5.1816" x2="5.1816" y2="-4.5212" width="0.1524" layer="21"/>
<wire x1="5.1816" y1="5.1816" x2="4.5212" y2="5.1816" width="0.1524" layer="21"/>
<wire x1="-5.1816" y1="5.1816" x2="-5.1816" y2="4.5212" width="0.1524" layer="21"/>
<wire x1="-5.1816" y1="-4.5212" x2="-5.1816" y2="-5.1816" width="0.1524" layer="21"/>
<wire x1="4.5212" y1="-5.1816" x2="5.1816" y2="-5.1816" width="0.1524" layer="21"/>
<wire x1="5.1816" y1="4.5212" x2="5.1816" y2="5.1816" width="0.1524" layer="21"/>
<wire x1="-4.5212" y1="5.1816" x2="-5.1816" y2="5.1816" width="0.1524" layer="21"/>
<polygon width="0.0254" layer="21">
<vertex x="-6.7056" y="-3.0095"/>
<vertex x="-6.7056" y="-3.3905"/>
<vertex x="-6.4516" y="-3.3905"/>
<vertex x="-6.4516" y="-3.0095"/>
</polygon>
<polygon width="0.0254" layer="21">
<vertex x="2.2095" y="-6.4516"/>
<vertex x="2.2095" y="-6.7056"/>
<vertex x="2.5905" y="-6.7056"/>
<vertex x="2.5905" y="-6.4516"/>
</polygon>
<polygon width="0.0254" layer="21">
<vertex x="6.7056" y="1.7905"/>
<vertex x="6.7056" y="1.4095"/>
<vertex x="6.4516" y="1.4095"/>
<vertex x="6.4516" y="1.7905"/>
</polygon>
<polygon width="0.0254" layer="21">
<vertex x="-0.9905" y="6.4516"/>
<vertex x="-0.9905" y="6.7056"/>
<vertex x="-0.6095" y="6.7056"/>
<vertex x="-0.6095" y="6.4516"/>
</polygon>
<text x="-7.4168" y="4.1402" size="1.27" layer="21" ratio="6" rot="SR0">*</text>
<text x="-1.7272" y="-0.635" size="1.27" layer="21" ratio="6" rot="SR0">&gt;Value</text>
<wire x1="3.81" y1="5.0546" x2="4.1656" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="4.1656" y1="5.0546" x2="4.1656" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="4.1656" y1="6.0452" x2="3.81" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="3.81" y1="6.0452" x2="3.81" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="3.0226" y1="5.0546" x2="3.3782" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="3.3782" y1="5.0546" x2="3.3782" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="3.3782" y1="6.0452" x2="3.0226" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="3.0226" y1="6.0452" x2="3.0226" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="2.2098" y1="5.0546" x2="2.5654" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="2.5654" y1="5.0546" x2="2.5654" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="2.5654" y1="6.0452" x2="2.2098" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="2.2098" y1="6.0452" x2="2.2098" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="1.4224" y1="5.0546" x2="1.778" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="1.778" y1="5.0546" x2="1.778" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="1.778" y1="6.0452" x2="1.4224" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="1.4224" y1="6.0452" x2="1.4224" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="0.6096" y1="5.0546" x2="0.9652" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="0.9652" y1="5.0546" x2="0.9652" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="0.9652" y1="6.0452" x2="0.6096" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="0.6096" y1="6.0452" x2="0.6096" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="-0.1778" y1="5.0546" x2="0.1778" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="0.1778" y1="5.0546" x2="0.1778" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="0.1778" y1="6.0452" x2="-0.1778" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="-0.1778" y1="6.0452" x2="-0.1778" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="-0.9652" y1="5.0546" x2="-0.6096" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="-0.6096" y1="5.0546" x2="-0.6096" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="-0.6096" y1="6.0452" x2="-0.9652" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="-0.9652" y1="6.0452" x2="-0.9652" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="-1.778" y1="5.0546" x2="-1.4224" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="-1.4224" y1="5.0546" x2="-1.4224" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="-1.4224" y1="6.0452" x2="-1.778" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="-1.778" y1="6.0452" x2="-1.778" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="-2.5654" y1="5.0546" x2="-2.2098" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="-2.2098" y1="5.0546" x2="-2.2098" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="-2.2098" y1="6.0452" x2="-2.5654" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="-2.5654" y1="6.0452" x2="-2.5654" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="-3.3782" y1="5.0546" x2="-3.0226" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="-3.0226" y1="5.0546" x2="-3.0226" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="-3.0226" y1="6.0452" x2="-3.3782" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="-3.3782" y1="6.0452" x2="-3.3782" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="-4.1656" y1="5.0546" x2="-3.81" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="-3.81" y1="5.0546" x2="-3.81" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="-3.81" y1="6.0452" x2="-4.1656" y2="6.0452" width="0.1524" layer="51"/>
<wire x1="-4.1656" y1="6.0452" x2="-4.1656" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="3.81" x2="-5.0546" y2="4.1656" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="4.1656" x2="-6.0452" y2="4.1656" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="4.1656" x2="-6.0452" y2="3.81" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="3.81" x2="-5.0546" y2="3.81" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="3.0226" x2="-5.0546" y2="3.3782" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="3.3782" x2="-6.0452" y2="3.3782" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="3.3782" x2="-6.0452" y2="3.0226" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="3.0226" x2="-5.0546" y2="3.0226" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="2.2098" x2="-5.0546" y2="2.5654" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="2.5654" x2="-6.0452" y2="2.5654" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="2.5654" x2="-6.0452" y2="2.2098" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="2.2098" x2="-5.0546" y2="2.2098" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="1.4224" x2="-5.0546" y2="1.778" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="1.778" x2="-6.0452" y2="1.778" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="1.778" x2="-6.0452" y2="1.4224" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="1.4224" x2="-5.0546" y2="1.4224" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="0.6096" x2="-5.0546" y2="0.9652" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="0.9652" x2="-6.0452" y2="0.9652" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="0.9652" x2="-6.0452" y2="0.6096" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="0.6096" x2="-5.0546" y2="0.6096" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="-0.1778" x2="-5.0546" y2="0.1778" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="0.1778" x2="-6.0452" y2="0.1778" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="0.1778" x2="-6.0452" y2="-0.1778" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="-0.1778" x2="-5.0546" y2="-0.1778" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="-0.9652" x2="-5.0546" y2="-0.6096" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="-0.6096" x2="-6.0452" y2="-0.6096" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="-0.6096" x2="-6.0452" y2="-0.9652" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="-0.9652" x2="-5.0546" y2="-0.9652" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="-1.778" x2="-5.0546" y2="-1.4224" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="-1.4224" x2="-6.0452" y2="-1.4224" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="-1.4224" x2="-6.0452" y2="-1.778" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="-1.778" x2="-5.0546" y2="-1.778" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="-2.5654" x2="-5.0546" y2="-2.2098" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="-2.2098" x2="-6.0452" y2="-2.2098" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="-2.2098" x2="-6.0452" y2="-2.5654" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="-2.5654" x2="-5.0546" y2="-2.5654" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="-3.3782" x2="-5.0546" y2="-3.0226" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="-3.0226" x2="-6.0452" y2="-3.0226" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="-3.0226" x2="-6.0452" y2="-3.3782" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="-3.3782" x2="-5.0546" y2="-3.3782" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="-4.1656" x2="-5.0546" y2="-3.81" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="-3.81" x2="-6.0452" y2="-3.81" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="-3.81" x2="-6.0452" y2="-4.1656" width="0.1524" layer="51"/>
<wire x1="-6.0452" y1="-4.1656" x2="-5.0546" y2="-4.1656" width="0.1524" layer="51"/>
<wire x1="-3.81" y1="-5.0546" x2="-4.1656" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="-4.1656" y1="-5.0546" x2="-4.1656" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="-4.1656" y1="-6.0452" x2="-3.81" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="-3.81" y1="-6.0452" x2="-3.81" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="-3.0226" y1="-5.0546" x2="-3.3782" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="-3.3782" y1="-5.0546" x2="-3.3782" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="-3.3782" y1="-6.0452" x2="-3.0226" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="-3.0226" y1="-6.0452" x2="-3.0226" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="-2.2098" y1="-5.0546" x2="-2.5654" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="-2.5654" y1="-5.0546" x2="-2.5654" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="-2.5654" y1="-6.0452" x2="-2.2098" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="-2.2098" y1="-6.0452" x2="-2.2098" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="-1.4224" y1="-5.0546" x2="-1.778" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="-1.778" y1="-5.0546" x2="-1.778" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="-1.778" y1="-6.0452" x2="-1.4224" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="-1.4224" y1="-6.0452" x2="-1.4224" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="-0.6096" y1="-5.0546" x2="-0.9652" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="-0.9652" y1="-5.0546" x2="-0.9652" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="-0.9652" y1="-6.0452" x2="-0.6096" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="-0.6096" y1="-6.0452" x2="-0.6096" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="0.1778" y1="-5.0546" x2="-0.1778" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="-0.1778" y1="-5.0546" x2="-0.1778" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="-0.1778" y1="-6.0452" x2="0.1778" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="0.1778" y1="-6.0452" x2="0.1778" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="0.9652" y1="-5.0546" x2="0.6096" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="0.6096" y1="-5.0546" x2="0.6096" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="0.6096" y1="-6.0452" x2="0.9652" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="0.9652" y1="-6.0452" x2="0.9652" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="1.778" y1="-5.0546" x2="1.4224" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="1.4224" y1="-5.0546" x2="1.4224" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="1.4224" y1="-6.0452" x2="1.778" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="1.778" y1="-6.0452" x2="1.778" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="2.5654" y1="-5.0546" x2="2.2098" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="2.2098" y1="-5.0546" x2="2.2098" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="2.2098" y1="-6.0452" x2="2.5654" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="2.5654" y1="-6.0452" x2="2.5654" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="3.3782" y1="-5.0546" x2="3.0226" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="3.0226" y1="-5.0546" x2="3.0226" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="3.0226" y1="-6.0452" x2="3.3782" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="3.3782" y1="-6.0452" x2="3.3782" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="4.1656" y1="-5.0546" x2="3.81" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="3.81" y1="-5.0546" x2="3.81" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="3.81" y1="-6.0452" x2="4.1656" y2="-6.0452" width="0.1524" layer="51"/>
<wire x1="4.1656" y1="-6.0452" x2="4.1656" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="-3.81" x2="5.0546" y2="-4.1656" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="-4.1656" x2="6.0452" y2="-4.1656" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="-4.1656" x2="6.0452" y2="-3.81" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="-3.81" x2="5.0546" y2="-3.81" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="-3.0226" x2="5.0546" y2="-3.3782" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="-3.3782" x2="6.0452" y2="-3.3782" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="-3.3782" x2="6.0452" y2="-3.0226" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="-3.0226" x2="5.0546" y2="-3.0226" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="-2.2098" x2="5.0546" y2="-2.5654" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="-2.5654" x2="6.0452" y2="-2.5654" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="-2.5654" x2="6.0452" y2="-2.2098" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="-2.2098" x2="5.0546" y2="-2.2098" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="-1.4224" x2="5.0546" y2="-1.778" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="-1.778" x2="6.0452" y2="-1.778" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="-1.778" x2="6.0452" y2="-1.4224" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="-1.4224" x2="5.0546" y2="-1.4224" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="-0.6096" x2="5.0546" y2="-0.9652" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="-0.9652" x2="6.0452" y2="-0.9652" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="-0.9652" x2="6.0452" y2="-0.6096" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="-0.6096" x2="5.0546" y2="-0.6096" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="0.1778" x2="5.0546" y2="-0.1778" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="-0.1778" x2="6.0452" y2="-0.1778" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="-0.1778" x2="6.0452" y2="0.1778" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="0.1778" x2="5.0546" y2="0.1778" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="0.9652" x2="5.0546" y2="0.6096" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="0.6096" x2="6.0452" y2="0.6096" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="0.6096" x2="6.0452" y2="0.9652" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="0.9652" x2="5.0546" y2="0.9652" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="1.778" x2="5.0546" y2="1.4224" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="1.4224" x2="6.0452" y2="1.4224" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="1.4224" x2="6.0452" y2="1.778" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="1.778" x2="5.0546" y2="1.778" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="2.5654" x2="5.0546" y2="2.2098" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="2.2098" x2="6.0452" y2="2.2098" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="2.2098" x2="6.0452" y2="2.5654" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="2.5654" x2="5.0546" y2="2.5654" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="3.3782" x2="5.0546" y2="3.0226" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="3.0226" x2="6.0452" y2="3.0226" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="3.0226" x2="6.0452" y2="3.3782" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="3.3782" x2="5.0546" y2="3.3782" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="4.1656" x2="5.0546" y2="3.81" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="3.81" x2="6.0452" y2="3.81" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="3.81" x2="6.0452" y2="4.1656" width="0.1524" layer="51"/>
<wire x1="6.0452" y1="4.1656" x2="5.0546" y2="4.1656" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="3.7846" x2="-3.7846" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="-5.0546" x2="5.0546" y2="-5.0546" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="-5.0546" x2="5.0546" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="5.0546" y1="5.0546" x2="-5.0546" y2="5.0546" width="0.1524" layer="51"/>
<wire x1="-5.0546" y1="5.0546" x2="-5.0546" y2="-5.0546" width="0.1524" layer="51"/>
<text x="-5.2578" y="3.7592" size="1.27" layer="51" ratio="6" rot="SR0">*</text>
<text x="-3.2766" y="-0.635" size="1.27" layer="25" ratio="6" rot="SR0">&gt;Name</text>
</package>
</packages>
<symbols>
<symbol name="ATMEGA32U4-AU">
<pin name="INT.6/AIN0)PE6" x="2.54" y="0" length="middle"/>
<pin name="UVCC" x="2.54" y="-2.54" length="middle" direction="pwr"/>
<pin name="D-" x="2.54" y="-5.08" length="middle" direction="pas"/>
<pin name="D+" x="2.54" y="-7.62" length="middle" direction="pas"/>
<pin name="UGND" x="2.54" y="-10.16" length="middle" direction="pwr"/>
<pin name="UCAP" x="2.54" y="-12.7" length="middle" direction="pwr"/>
<pin name="VBUS" x="2.54" y="-15.24" length="middle" direction="pwr"/>
<pin name="(SS/PCINT0)_PB0" x="2.54" y="-17.78" length="middle"/>
<pin name="(PCINT1/SCLK)_PB1" x="2.54" y="-20.32" length="middle"/>
<pin name="(PDI/PCINT2/MOSI)_PB2" x="2.54" y="-22.86" length="middle"/>
<pin name="(PDO/PCINT3/MISO)_PB3" x="2.54" y="-25.4" length="middle"/>
<pin name="(PCINT7/OC0A/OC1C/RTS!)_PB7" x="2.54" y="-27.94" length="middle"/>
<pin name="!" x="2.54" y="-30.48" length="middle" direction="in"/>
<pin name="VCC_2" x="2.54" y="-33.02" length="middle" direction="pwr"/>
<pin name="GND_2" x="2.54" y="-35.56" length="middle" direction="pwr"/>
<pin name="XTAL2" x="2.54" y="-38.1" length="middle" direction="out"/>
<pin name="XTAL1" x="2.54" y="-40.64" length="middle" direction="in"/>
<pin name="(OC0B/SCL/INT0)_PD0" x="2.54" y="-43.18" length="middle"/>
<pin name="(SDA/INT1)_PD1" x="2.54" y="-45.72" length="middle"/>
<pin name="(RXD1/INT2)_PD2" x="2.54" y="-48.26" length="middle"/>
<pin name="(TXD1/INT3)_PD3" x="2.54" y="-50.8" length="middle"/>
<pin name="(XCK1/CTS!)_PD5" x="2.54" y="-53.34" length="middle"/>
<pin name="GND_3" x="139.7" y="-53.34" length="middle" direction="pwr" rot="R180"/>
<pin name="AVCC_2" x="139.7" y="-50.8" length="middle" direction="pwr" rot="R180"/>
<pin name="PD4_(ICP1/ADC8" x="139.7" y="-48.26" length="middle" rot="R180"/>
<pin name="PD6_(T1/OC4D/ADC9)" x="139.7" y="-45.72" length="middle" rot="R180"/>
<pin name="PD7_(T0/OC4D/ADC10)" x="139.7" y="-43.18" length="middle" rot="R180"/>
<pin name="PB4_(PCINT4/ADC11)" x="139.7" y="-40.64" length="middle" rot="R180"/>
<pin name="PB5_(PCINT5/OC1A/OC4B/ADC12)" x="139.7" y="-38.1" length="middle" rot="R180"/>
<pin name="PB6_(PCINT6/OC1B/OC4B/ADC13)" x="139.7" y="-35.56" length="middle" rot="R180"/>
<pin name="PC6_(OC3A/OC4A!)" x="139.7" y="-33.02" length="middle" rot="R180"/>
<pin name="PC7_(ICP3/CLK0/OC4A)" x="139.7" y="-30.48" length="middle" rot="R180"/>
<pin name="PE2_(HWB!)" x="139.7" y="-27.94" length="middle" rot="R180"/>
<pin name="VCC" x="139.7" y="-25.4" length="middle" direction="pwr" rot="R180"/>
<pin name="GND_4" x="139.7" y="-22.86" length="middle" direction="pwr" rot="R180"/>
<pin name="PF7_(ADC7/TDI)" x="139.7" y="-20.32" length="middle" rot="R180"/>
<pin name="PF6_(ADC6/TDO)" x="139.7" y="-17.78" length="middle" rot="R180"/>
<pin name="PF5_(ADC5/TMS)" x="139.7" y="-15.24" length="middle" rot="R180"/>
<pin name="PF4_(ADC4/TCK)" x="139.7" y="-12.7" length="middle" rot="R180"/>
<pin name="PF1_(ADC1)" x="139.7" y="-10.16" length="middle" rot="R180"/>
<pin name="PF0_(ADC0)" x="139.7" y="-7.62" length="middle" rot="R180"/>
<pin name="AREF" x="139.7" y="-5.08" length="middle" direction="pas" rot="R180"/>
<pin name="GND" x="139.7" y="-2.54" length="middle" direction="pwr" rot="R180"/>
<pin name="AVCC" x="139.7" y="0" length="middle" direction="pwr" rot="R180"/>
<wire x1="7.62" y1="5.08" x2="7.62" y2="-58.42" width="0.1524" layer="94"/>
<wire x1="7.62" y1="-58.42" x2="134.62" y2="-58.42" width="0.1524" layer="94"/>
<wire x1="134.62" y1="-58.42" x2="134.62" y2="5.08" width="0.1524" layer="94"/>
<wire x1="134.62" y1="5.08" x2="7.62" y2="5.08" width="0.1524" layer="94"/>
<text x="66.3956" y="9.1186" size="2.0828" layer="95" ratio="6" rot="SR0">&gt;Name</text>
<text x="65.7606" y="6.5786" size="2.0828" layer="96" ratio="6" rot="SR0">&gt;Value</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="ATMEGA32U4-AU" prefix="U">
<gates>
<gate name="A" symbol="ATMEGA32U4-AU" x="0" y="0"/>
</gates>
<devices>
<device name="" package="44ML_ATM">
<connects>
<connect gate="A" pin="!" pad="13"/>
<connect gate="A" pin="(OC0B/SCL/INT0)_PD0" pad="18"/>
<connect gate="A" pin="(PCINT1/SCLK)_PB1" pad="9"/>
<connect gate="A" pin="(PCINT7/OC0A/OC1C/RTS!)_PB7" pad="12"/>
<connect gate="A" pin="(PDI/PCINT2/MOSI)_PB2" pad="10"/>
<connect gate="A" pin="(PDO/PCINT3/MISO)_PB3" pad="11"/>
<connect gate="A" pin="(RXD1/INT2)_PD2" pad="20"/>
<connect gate="A" pin="(SDA/INT1)_PD1" pad="19"/>
<connect gate="A" pin="(SS/PCINT0)_PB0" pad="8"/>
<connect gate="A" pin="(TXD1/INT3)_PD3" pad="21"/>
<connect gate="A" pin="(XCK1/CTS!)_PD5" pad="22"/>
<connect gate="A" pin="AREF" pad="42"/>
<connect gate="A" pin="AVCC" pad="44"/>
<connect gate="A" pin="AVCC_2" pad="24"/>
<connect gate="A" pin="D+" pad="4"/>
<connect gate="A" pin="D-" pad="3"/>
<connect gate="A" pin="GND" pad="43"/>
<connect gate="A" pin="GND_2" pad="15"/>
<connect gate="A" pin="GND_3" pad="23"/>
<connect gate="A" pin="GND_4" pad="35"/>
<connect gate="A" pin="INT.6/AIN0)PE6" pad="1"/>
<connect gate="A" pin="PB4_(PCINT4/ADC11)" pad="28"/>
<connect gate="A" pin="PB5_(PCINT5/OC1A/OC4B/ADC12)" pad="29"/>
<connect gate="A" pin="PB6_(PCINT6/OC1B/OC4B/ADC13)" pad="30"/>
<connect gate="A" pin="PC6_(OC3A/OC4A!)" pad="31"/>
<connect gate="A" pin="PC7_(ICP3/CLK0/OC4A)" pad="32"/>
<connect gate="A" pin="PD4_(ICP1/ADC8" pad="25"/>
<connect gate="A" pin="PD6_(T1/OC4D/ADC9)" pad="26"/>
<connect gate="A" pin="PD7_(T0/OC4D/ADC10)" pad="27"/>
<connect gate="A" pin="PE2_(HWB!)" pad="33"/>
<connect gate="A" pin="PF0_(ADC0)" pad="41"/>
<connect gate="A" pin="PF1_(ADC1)" pad="40"/>
<connect gate="A" pin="PF4_(ADC4/TCK)" pad="39"/>
<connect gate="A" pin="PF5_(ADC5/TMS)" pad="38"/>
<connect gate="A" pin="PF6_(ADC6/TDO)" pad="37"/>
<connect gate="A" pin="PF7_(ADC7/TDI)" pad="36"/>
<connect gate="A" pin="UCAP" pad="6"/>
<connect gate="A" pin="UGND" pad="5"/>
<connect gate="A" pin="UVCC" pad="2"/>
<connect gate="A" pin="VBUS" pad="7"/>
<connect gate="A" pin="VCC" pad="34"/>
<connect gate="A" pin="VCC_2" pad="14"/>
<connect gate="A" pin="XTAL1" pad="17"/>
<connect gate="A" pin="XTAL2" pad="16"/>
</connects>
<technologies>
<technology name="">
<attribute name="BUILT_BY" value="EMA_UL_Team" constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="ATMEGA32U4-AU" constant="no"/>
<attribute name="SOURCELIBRARY" value="Atmel_2011-04-02" constant="no"/>
<attribute name="VENDOR" value="Atmel" constant="no"/>
</technology>
</technologies>
</device>
<device name="44ML_ATM-M" package="44ML_ATM-M">
<connects>
<connect gate="A" pin="!" pad="13"/>
<connect gate="A" pin="(OC0B/SCL/INT0)_PD0" pad="18"/>
<connect gate="A" pin="(PCINT1/SCLK)_PB1" pad="9"/>
<connect gate="A" pin="(PCINT7/OC0A/OC1C/RTS!)_PB7" pad="12"/>
<connect gate="A" pin="(PDI/PCINT2/MOSI)_PB2" pad="10"/>
<connect gate="A" pin="(PDO/PCINT3/MISO)_PB3" pad="11"/>
<connect gate="A" pin="(RXD1/INT2)_PD2" pad="20"/>
<connect gate="A" pin="(SDA/INT1)_PD1" pad="19"/>
<connect gate="A" pin="(SS/PCINT0)_PB0" pad="8"/>
<connect gate="A" pin="(TXD1/INT3)_PD3" pad="21"/>
<connect gate="A" pin="(XCK1/CTS!)_PD5" pad="22"/>
<connect gate="A" pin="AREF" pad="42"/>
<connect gate="A" pin="AVCC" pad="44"/>
<connect gate="A" pin="AVCC_2" pad="24"/>
<connect gate="A" pin="D+" pad="4"/>
<connect gate="A" pin="D-" pad="3"/>
<connect gate="A" pin="GND" pad="43"/>
<connect gate="A" pin="GND_2" pad="15"/>
<connect gate="A" pin="GND_3" pad="23"/>
<connect gate="A" pin="GND_4" pad="35"/>
<connect gate="A" pin="INT.6/AIN0)PE6" pad="1"/>
<connect gate="A" pin="PB4_(PCINT4/ADC11)" pad="28"/>
<connect gate="A" pin="PB5_(PCINT5/OC1A/OC4B/ADC12)" pad="29"/>
<connect gate="A" pin="PB6_(PCINT6/OC1B/OC4B/ADC13)" pad="30"/>
<connect gate="A" pin="PC6_(OC3A/OC4A!)" pad="31"/>
<connect gate="A" pin="PC7_(ICP3/CLK0/OC4A)" pad="32"/>
<connect gate="A" pin="PD4_(ICP1/ADC8" pad="25"/>
<connect gate="A" pin="PD6_(T1/OC4D/ADC9)" pad="26"/>
<connect gate="A" pin="PD7_(T0/OC4D/ADC10)" pad="27"/>
<connect gate="A" pin="PE2_(HWB!)" pad="33"/>
<connect gate="A" pin="PF0_(ADC0)" pad="41"/>
<connect gate="A" pin="PF1_(ADC1)" pad="40"/>
<connect gate="A" pin="PF4_(ADC4/TCK)" pad="39"/>
<connect gate="A" pin="PF5_(ADC5/TMS)" pad="38"/>
<connect gate="A" pin="PF6_(ADC6/TDO)" pad="37"/>
<connect gate="A" pin="PF7_(ADC7/TDI)" pad="36"/>
<connect gate="A" pin="UCAP" pad="6"/>
<connect gate="A" pin="UGND" pad="5"/>
<connect gate="A" pin="UVCC" pad="2"/>
<connect gate="A" pin="VBUS" pad="7"/>
<connect gate="A" pin="VCC" pad="34"/>
<connect gate="A" pin="VCC_2" pad="14"/>
<connect gate="A" pin="XTAL1" pad="17"/>
<connect gate="A" pin="XTAL2" pad="16"/>
</connects>
<technologies>
<technology name="">
<attribute name="BUILT_BY" value="EMA_UL_Team" constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="ATMEGA32U4-AU" constant="no"/>
<attribute name="SOURCELIBRARY" value="Atmel_2011-04-02" constant="no"/>
<attribute name="VENDOR" value="Atmel" constant="no"/>
</technology>
</technologies>
</device>
<device name="44ML_ATM-L" package="44ML_ATM-L">
<connects>
<connect gate="A" pin="!" pad="13"/>
<connect gate="A" pin="(OC0B/SCL/INT0)_PD0" pad="18"/>
<connect gate="A" pin="(PCINT1/SCLK)_PB1" pad="9"/>
<connect gate="A" pin="(PCINT7/OC0A/OC1C/RTS!)_PB7" pad="12"/>
<connect gate="A" pin="(PDI/PCINT2/MOSI)_PB2" pad="10"/>
<connect gate="A" pin="(PDO/PCINT3/MISO)_PB3" pad="11"/>
<connect gate="A" pin="(RXD1/INT2)_PD2" pad="20"/>
<connect gate="A" pin="(SDA/INT1)_PD1" pad="19"/>
<connect gate="A" pin="(SS/PCINT0)_PB0" pad="8"/>
<connect gate="A" pin="(TXD1/INT3)_PD3" pad="21"/>
<connect gate="A" pin="(XCK1/CTS!)_PD5" pad="22"/>
<connect gate="A" pin="AREF" pad="42"/>
<connect gate="A" pin="AVCC" pad="44"/>
<connect gate="A" pin="AVCC_2" pad="24"/>
<connect gate="A" pin="D+" pad="4"/>
<connect gate="A" pin="D-" pad="3"/>
<connect gate="A" pin="GND" pad="43"/>
<connect gate="A" pin="GND_2" pad="15"/>
<connect gate="A" pin="GND_3" pad="23"/>
<connect gate="A" pin="GND_4" pad="35"/>
<connect gate="A" pin="INT.6/AIN0)PE6" pad="1"/>
<connect gate="A" pin="PB4_(PCINT4/ADC11)" pad="28"/>
<connect gate="A" pin="PB5_(PCINT5/OC1A/OC4B/ADC12)" pad="29"/>
<connect gate="A" pin="PB6_(PCINT6/OC1B/OC4B/ADC13)" pad="30"/>
<connect gate="A" pin="PC6_(OC3A/OC4A!)" pad="31"/>
<connect gate="A" pin="PC7_(ICP3/CLK0/OC4A)" pad="32"/>
<connect gate="A" pin="PD4_(ICP1/ADC8" pad="25"/>
<connect gate="A" pin="PD6_(T1/OC4D/ADC9)" pad="26"/>
<connect gate="A" pin="PD7_(T0/OC4D/ADC10)" pad="27"/>
<connect gate="A" pin="PE2_(HWB!)" pad="33"/>
<connect gate="A" pin="PF0_(ADC0)" pad="41"/>
<connect gate="A" pin="PF1_(ADC1)" pad="40"/>
<connect gate="A" pin="PF4_(ADC4/TCK)" pad="39"/>
<connect gate="A" pin="PF5_(ADC5/TMS)" pad="38"/>
<connect gate="A" pin="PF6_(ADC6/TDO)" pad="37"/>
<connect gate="A" pin="PF7_(ADC7/TDI)" pad="36"/>
<connect gate="A" pin="UCAP" pad="6"/>
<connect gate="A" pin="UGND" pad="5"/>
<connect gate="A" pin="UVCC" pad="2"/>
<connect gate="A" pin="VBUS" pad="7"/>
<connect gate="A" pin="VCC" pad="34"/>
<connect gate="A" pin="VCC_2" pad="14"/>
<connect gate="A" pin="XTAL1" pad="17"/>
<connect gate="A" pin="XTAL2" pad="16"/>
</connects>
<technologies>
<technology name="">
<attribute name="BUILT_BY" value="EMA_UL_Team" constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="ATMEGA32U4-AU" constant="no"/>
<attribute name="SOURCELIBRARY" value="Atmel_2011-04-02" constant="no"/>
<attribute name="VENDOR" value="Atmel" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="JST">
<packages>
<package name="JST_SM03B-SRSS-TB(LF)(SN)">
<wire x1="-2.5" y1="-0.325" x2="2.5" y2="-0.325" width="0.127" layer="51"/>
<wire x1="2.5" y1="-0.325" x2="2.5" y2="-4.575" width="0.127" layer="51"/>
<wire x1="2.5" y1="-4.575" x2="-2.5" y2="-4.575" width="0.127" layer="51"/>
<wire x1="-2.5" y1="-4.575" x2="-2.5" y2="-0.325" width="0.127" layer="51"/>
<wire x1="-1.615" y1="-0.325" x2="-2.5" y2="-0.325" width="0.127" layer="21"/>
<wire x1="-2.5" y1="-0.325" x2="-2.5" y2="-2.6" width="0.127" layer="21"/>
<wire x1="-1.3" y1="-4.575" x2="1.3" y2="-4.575" width="0.127" layer="21"/>
<wire x1="2.5" y1="-2.6" x2="2.5" y2="-0.325" width="0.127" layer="21"/>
<wire x1="2.5" y1="-0.325" x2="1.615" y2="-0.325" width="0.127" layer="21"/>
<circle x="-1.8" y="0.4" radius="0.1" width="0.3" layer="21"/>
<circle x="-1.8" y="0.4" radius="0.1" width="0.3" layer="51"/>
<wire x1="-3.15" y1="1.025" x2="3.15" y2="1.025" width="0.05" layer="39"/>
<wire x1="3.15" y1="1.025" x2="3.15" y2="-5.025" width="0.05" layer="39"/>
<wire x1="3.15" y1="-5.025" x2="-3.15" y2="-5.025" width="0.05" layer="39"/>
<wire x1="-3.15" y1="-5.025" x2="-3.15" y2="1.025" width="0.05" layer="39"/>
<text x="-3.81" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-3.81" y="-6.35" size="1.27" layer="27">&gt;VALUE</text>
<smd name="2" x="0" y="0" dx="1.55" dy="0.6" layer="1" rot="R90"/>
<smd name="1" x="-1" y="0" dx="1.55" dy="0.6" layer="1" rot="R90"/>
<smd name="3" x="1" y="0" dx="1.55" dy="0.6" layer="1" rot="R90"/>
<smd name="S1" x="-2.3" y="-3.875" dx="1.2" dy="1.8" layer="1" rot="R180"/>
<smd name="S2" x="2.3" y="-3.875" dx="1.2" dy="1.8" layer="1" rot="R180"/>
</package>
</packages>
<symbols>
<symbol name="SM03B-SRSS-TB(LF)(SN)">
<wire x1="-5.08" y1="7.62" x2="5.08" y2="7.62" width="0.1524" layer="94"/>
<wire x1="5.08" y1="7.62" x2="5.08" y2="-7.62" width="0.1524" layer="94"/>
<wire x1="5.08" y1="-7.62" x2="-5.08" y2="-7.62" width="0.1524" layer="94"/>
<wire x1="-5.08" y1="-7.62" x2="-5.08" y2="7.62" width="0.1524" layer="94"/>
<text x="-5.08" y="8.382" size="1.778" layer="95">&gt;NAME</text>
<text x="-5.08" y="-10.16" size="1.778" layer="96">&gt;VALUE</text>
<pin name="1" x="-10.16" y="5.08" length="middle" direction="pas"/>
<pin name="2" x="-10.16" y="2.54" length="middle" direction="pas"/>
<pin name="3" x="-10.16" y="0" length="middle" direction="pas"/>
<pin name="SHIELD" x="-10.16" y="-5.08" length="middle" direction="pas"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="SM03B-SRSS-TB(LF)(SN)" prefix="J">
<description>SH Series 3 Position 1 mm Pitch Surface Mount Side Entry Shrouded Header </description>
<gates>
<gate name="G$1" symbol="SM03B-SRSS-TB(LF)(SN)" x="0" y="0"/>
</gates>
<devices>
<device name="" package="JST_SM03B-SRSS-TB(LF)(SN)">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
<connect gate="G$1" pin="SHIELD" pad="S1 S2"/>
</connects>
<technologies>
<technology name="">
<attribute name="DESCRIPTION" value=" Connector Header Surface Mount, Right Angle 3 position 0.039 (1.00mm) "/>
<attribute name="DIGI-KEY_PART_NUMBER" value="455-1803-1-ND"/>
<attribute name="MF" value="JST Sales"/>
<attribute name="MP" value="SM03B-SRSS-TB(LF)(SN)"/>
<attribute name="PACKAGE" value="None"/>
<attribute name="PURCHASE-URL" value="https://pricing.snapeda.com/search/part/SM03B-SRSS-TB(LF)(SN)/?ref=eda"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
</libraries>
<attributes>
</attributes>
<variantdefs>
</variantdefs>
<classes>
<class number="0" name="default" width="0" drill="0">
</class>
</classes>
<parts>
<part name="GND14" library="supply1" deviceset="GND" device=""/>
<part name="SV1" library="filip" deviceset="MA03-2" device="SMD"/>
<part name="R5" library="rcl" deviceset="R-US_" device="R0603" value="1k"/>
<part name="GND24" library="supply1" deviceset="GND" device=""/>
<part name="C5" library="cap-master" deviceset="CNP-" device="0805" value="10uF"/>
<part name="R7" library="rcl" deviceset="R-US_" device="R0603" value="1k"/>
<part name="GND20" library="supply1" deviceset="GND" device=""/>
<part name="P+10" library="supply1" deviceset="+5V" device=""/>
<part name="P+2" library="supply1" deviceset="+5V" device=""/>
<part name="GND7" library="supply1" deviceset="GND" device=""/>
<part name="R8" library="rcl" deviceset="R-US_" device="R0603" value="680"/>
<part name="H1" library="filip" deviceset="MOUNT-HOLE" device=""/>
<part name="H2" library="filip" deviceset="MOUNT-HOLE" device=""/>
<part name="GND19" library="supply1" deviceset="GND" device=""/>
<part name="H3" library="filip" deviceset="MOUNT-HOLE" device=""/>
<part name="JP1" library="filip" deviceset="M02" device="2.54MM_SCREWTERM"/>
<part name="JP2" library="filip" deviceset="M03" device="SCREW2.54"/>
<part name="PAD1" library="wirepad" deviceset="WIREPAD" device="1,6/0,8" value=""/>
<part name="IC5" library="filip" deviceset="MCP6001" device="LT"/>
<part name="LED1" library="SamacSys_Parts" deviceset="150080RS75000" device=""/>
<part name="Y1" library="SamacSys_Parts" deviceset="TSX-3225_16.0000MF18X-AC3" device=""/>
<part name="IC1" library="SamacSys_Parts" deviceset="MIC5365-3.0YC5_TR" device="" value="MIC5365-3.3YC5_TR"/>
<part name="IC4" library="filip" deviceset="MLX75306" device=""/>
<part name="J1" library="USB_jacks" deviceset="ZX62D-B-5PA8(30)" device="" value="USB"/>
<part name="U1" library="ATMEGA32U4" deviceset="ATMEGA32U4-AU" device=""/>
<part name="GND6" library="supply1" deviceset="GND" device=""/>
<part name="GND8" library="supply1" deviceset="GND" device=""/>
<part name="C4" library="cap-master" deviceset="CNP-" device="0603" value="18pF"/>
<part name="C8" library="cap-master" deviceset="CNP-" device="0603" value="18pF"/>
<part name="GND12" library="supply1" deviceset="GND" device=""/>
<part name="GND15" library="supply1" deviceset="GND" device=""/>
<part name="GND16" library="supply1" deviceset="GND" device=""/>
<part name="GND17" library="supply1" deviceset="GND" device=""/>
<part name="P+4" library="supply1" deviceset="+5V" device=""/>
<part name="P+6" library="supply1" deviceset="+5V" device=""/>
<part name="P+7" library="supply1" deviceset="+5V" device=""/>
<part name="P+8" library="supply1" deviceset="+5V" device=""/>
<part name="P+5" library="supply1" deviceset="VCC" device=""/>
<part name="GND13" library="supply1" deviceset="GND" device=""/>
<part name="C9" library="cap-master" deviceset="CNP-" device="0603" value="1uF"/>
<part name="GND11" library="supply1" deviceset="GND" device=""/>
<part name="R3" library="rcl" deviceset="R-US_" device="R0603" value="22"/>
<part name="R2" library="rcl" deviceset="R-US_" device="R0603" value="22"/>
<part name="P+3" library="supply1" deviceset="VCC" device=""/>
<part name="C7" library="cap-master" deviceset="CNP-" device="0603" value="1uF"/>
<part name="GND9" library="supply1" deviceset="GND" device=""/>
<part name="C10" library="cap-master" deviceset="CNP-" device="0603" value="0.1uF"/>
<part name="GND5" library="supply1" deviceset="GND" device=""/>
<part name="+3V32" library="supply1" deviceset="+3V3" device=""/>
<part name="GND1" library="supply1" deviceset="GND" device=""/>
<part name="P+1" library="supply1" deviceset="+5V" device=""/>
<part name="+3V31" library="supply1" deviceset="+3V3" device=""/>
<part name="C1" library="cap-master" deviceset="CNP-" device="0603" value="1uF"/>
<part name="C2" library="cap-master" deviceset="CNP-" device="0603" value="1uF"/>
<part name="GND3" library="supply1" deviceset="GND" device=""/>
<part name="GND22" library="supply1" deviceset="GND" device=""/>
<part name="GND21" library="supply1" deviceset="GND" device=""/>
<part name="P+9" library="supply1" deviceset="+5V" device=""/>
<part name="GND23" library="supply1" deviceset="GND" device=""/>
<part name="R6" library="rcl" deviceset="R-US_" device="R0603" value="100k"/>
<part name="GND18" library="supply1" deviceset="GND" device=""/>
<part name="R1" library="rcl" deviceset="R-US_" device="R0603" value="1k"/>
<part name="C6" library="cap-master" deviceset="CNP-" device="0603" value="0.1uF"/>
<part name="C13" library="cap-master" deviceset="CNP-" device="0805" value="10uF"/>
<part name="C11" library="cap-master" deviceset="CNP-" device="0603" value="1uF"/>
<part name="C12" library="cap-master" deviceset="CNP-" device="0603" value="0.01uF"/>
<part name="R4" library="rcl" deviceset="R-US_" device="R0603" value="10k"/>
<part name="F1" library="SamacSys_Parts" deviceset="0ZCJ0025FF2E" device="" value="500ma"/>
<part name="S1" library="SamacSys_Parts" deviceset="KMR631NGULCLFS" device=""/>
<part name="GND10" library="supply1" deviceset="GND" device=""/>
<part name="IC2" library="SamacSys_Parts" deviceset="74LVC2G34GW,125" device=""/>
<part name="IC3" library="SamacSys_Parts" deviceset="74LVC2G34GW,125" device=""/>
<part name="GND2" library="supply1" deviceset="GND" device=""/>
<part name="+3V33" library="supply1" deviceset="+3V3" device=""/>
<part name="J2" library="JST" deviceset="SM03B-SRSS-TB(LF)(SN)" device=""/>
<part name="C3" library="cap-master" deviceset="CNP-" device="0603" value="0.1uF"/>
<part name="GND4" library="supply1" deviceset="GND" device=""/>
<part name="GND25" library="supply1" deviceset="GND" device=""/>
<part name="GND26" library="supply1" deviceset="GND" device=""/>
<part name="R9" library="rcl" deviceset="R-US_" device="R0603" value="10k"/>
<part name="C14" library="cap-master" deviceset="CNP-" device="0603" value="0.1uF"/>
<part name="GND27" library="supply1" deviceset="GND" device=""/>
</parts>
<sheets>
<sheet>
<plain>
<text x="17.78" y="-137.16" size="5.08" layer="97">Filament Width Sensor V3.11</text>
<text x="106.68" y="-142.24" size="2.54" layer="97">Filip Mulier</text>
<text x="5.08" y="-5.08" size="1.778" layer="95">Pull POW_EN to ground 
when programming
ISC port.</text>
<text x="-5.08" y="-119.38" size="1.778" layer="95">2nd Order Lowpass 140Hz</text>
</plain>
<instances>
<instance part="GND14" gate="1" x="22.86" y="-17.78"/>
<instance part="SV1" gate="1" x="7.62" y="-12.7" rot="R180"/>
<instance part="R5" gate="G$1" x="2.54" y="-99.06" smashed="yes" rot="R180">
<attribute name="NAME" x="3.556" y="-91.4146" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="8.636" y="-93.98" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="GND24" gate="1" x="78.74" y="-111.76"/>
<instance part="C5" gate="G$1" x="-116.84" y="71.12" smashed="yes" rot="R90">
<attribute name="NAME" x="-121.285" y="75.565" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="-119.38" y="72.7075" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R7" gate="G$1" x="22.86" y="-63.5" smashed="yes" rot="R180">
<attribute name="NAME" x="26.416" y="-68.5546" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="26.416" y="-66.04" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="GND20" gate="1" x="35.56" y="-73.66"/>
<instance part="P+10" gate="1" x="78.74" y="-91.44"/>
<instance part="P+2" gate="1" x="-109.22" y="88.9"/>
<instance part="GND7" gate="1" x="-109.22" y="55.88"/>
<instance part="R8" gate="G$1" x="27.94" y="-43.18" rot="R180"/>
<instance part="H1" gate="G$1" x="106.68" y="-88.9"/>
<instance part="H2" gate="G$1" x="106.68" y="-96.52"/>
<instance part="GND19" gate="1" x="114.3" y="-48.26"/>
<instance part="H3" gate="G$1" x="106.68" y="-106.68"/>
<instance part="JP1" gate="G$1" x="53.34" y="-63.5" smashed="yes" rot="R180">
<attribute name="NAME" x="55.88" y="-69.342" size="1.778" layer="95" rot="R180"/>
</instance>
<instance part="JP2" gate="G$1" x="86.36" y="-101.6" rot="R180"/>
<instance part="PAD1" gate="G$1" x="45.72" y="-83.82" rot="R180"/>
<instance part="IC5" gate="G$1" x="53.34" y="-101.6"/>
<instance part="LED1" gate="G$1" x="50.8" y="-43.18" rot="R180"/>
<instance part="Y1" gate="G$1" x="-127" y="35.56"/>
<instance part="IC1" gate="G$1" x="-129.54" y="-12.7"/>
<instance part="IC4" gate="G$1" x="-53.34" y="-15.24"/>
<instance part="J1" gate="A" x="-81.28" y="71.12" rot="R90"/>
<instance part="U1" gate="A" x="-30.48" y="71.12"/>
<instance part="GND6" gate="1" x="-152.4" y="27.94"/>
<instance part="GND8" gate="1" x="-96.52" y="15.24"/>
<instance part="C4" gate="G$1" x="-129.54" y="30.48" smashed="yes" rot="R180">
<attribute name="NAME" x="-133.985" y="26.035" size="1.778" layer="95" rot="R270"/>
<attribute name="VALUE" x="-131.1275" y="27.94" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="C8" gate="G$1" x="-91.44" y="33.02" smashed="yes" rot="R270">
<attribute name="NAME" x="-86.995" y="28.575" size="1.778" layer="95"/>
<attribute name="VALUE" x="-88.9" y="31.4325" size="1.778" layer="96"/>
</instance>
<instance part="GND12" gate="1" x="-50.8" y="33.02"/>
<instance part="GND15" gate="1" x="111.76" y="15.24"/>
<instance part="GND16" gate="1" x="124.46" y="40.64"/>
<instance part="GND17" gate="1" x="127" y="66.04"/>
<instance part="P+4" gate="1" x="-50.8" y="43.18"/>
<instance part="P+6" gate="1" x="111.76" y="78.74"/>
<instance part="P+7" gate="1" x="121.92" y="53.34"/>
<instance part="P+8" gate="1" x="124.46" y="25.4"/>
<instance part="P+5" gate="VCC" x="-45.72" y="83.82"/>
<instance part="GND13" gate="1" x="-43.18" y="58.42"/>
<instance part="C9" gate="G$1" x="-35.56" y="58.42" smashed="yes">
<attribute name="NAME" x="-31.115" y="62.865" size="1.778" layer="95" rot="R90"/>
<attribute name="VALUE" x="-33.9725" y="60.96" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND11" gate="1" x="-66.04" y="68.58"/>
<instance part="R3" gate="G$1" x="-76.2" y="63.5" smashed="yes" rot="R270">
<attribute name="NAME" x="-74.422" y="63.4746" size="1.778" layer="95"/>
<attribute name="VALUE" x="-74.676" y="60.96" size="1.778" layer="96"/>
</instance>
<instance part="R2" gate="G$1" x="-78.74" y="63.5" smashed="yes" rot="R270">
<attribute name="NAME" x="-82.042" y="65.3034" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="-84.836" y="60.96" size="1.778" layer="96"/>
</instance>
<instance part="P+3" gate="VCC" x="-91.44" y="88.9"/>
<instance part="C7" gate="G$1" x="-91.44" y="68.58" smashed="yes" rot="R90">
<attribute name="NAME" x="-95.885" y="73.025" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="-93.98" y="70.1675" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="GND9" gate="1" x="-91.44" y="55.88"/>
<instance part="C10" gate="G$1" x="114.3" y="66.04" smashed="yes">
<attribute name="NAME" x="118.745" y="70.485" size="1.778" layer="95" rot="R90"/>
<attribute name="VALUE" x="115.8875" y="68.58" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND5" gate="1" x="-81.28" y="-45.72"/>
<instance part="+3V32" gate="G$1" x="-86.36" y="-12.7"/>
<instance part="GND1" gate="1" x="-142.24" y="-38.1"/>
<instance part="P+1" gate="1" x="-142.24" y="5.08"/>
<instance part="+3V31" gate="G$1" x="-99.06" y="2.54"/>
<instance part="C1" gate="G$1" x="-142.24" y="-12.7" smashed="yes" rot="R90">
<attribute name="NAME" x="-146.685" y="-8.255" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="-144.78" y="-11.1125" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="C2" gate="G$1" x="-99.06" y="-17.78" smashed="yes" rot="R90">
<attribute name="NAME" x="-93.345" y="-13.335" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="-93.98" y="-21.2725" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="GND3" gate="1" x="-99.06" y="-30.48"/>
<instance part="GND22" gate="1" x="60.96" y="-45.72"/>
<instance part="GND21" gate="1" x="53.34" y="-127"/>
<instance part="P+9" gate="1" x="53.34" y="-81.28"/>
<instance part="GND23" gate="1" x="66.04" y="-93.98"/>
<instance part="R6" gate="G$1" x="20.32" y="-99.06" smashed="yes" rot="R180">
<attribute name="NAME" x="18.796" y="-93.9546" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="28.956" y="-93.98" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="GND18" gate="1" x="27.94" y="-106.68"/>
<instance part="R1" gate="G$1" x="-101.6" y="-106.68" smashed="yes" rot="R180">
<attribute name="NAME" x="-98.044" y="-111.7346" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="-98.044" y="-109.22" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="C6" gate="G$1" x="-109.22" y="71.12" smashed="yes" rot="R90">
<attribute name="NAME" x="-103.505" y="70.485" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="-99.06" y="67.6275" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="C13" gate="G$1" x="35.56" y="-66.04" smashed="yes" rot="R90">
<attribute name="NAME" x="41.275" y="-64.135" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="43.18" y="-69.5325" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="C11" gate="G$1" x="10.16" y="-101.6" smashed="yes" rot="R90">
<attribute name="NAME" x="5.715" y="-102.235" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="7.62" y="-105.0925" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="C12" gate="G$1" x="27.94" y="-101.6" smashed="yes" rot="R90">
<attribute name="NAME" x="36.195" y="-102.235" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="43.18" y="-100.0125" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R4" gate="G$1" x="-43.18" y="40.64" smashed="yes" rot="R180">
<attribute name="NAME" x="-47.244" y="45.7454" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="-37.084" y="45.72" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="F1" gate="G$1" x="-109.22" y="78.74" smashed="yes">
<attribute name="NAME" x="-102.87" y="82.55" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-102.87" y="74.93" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="S1" gate="G$1" x="109.22" y="-45.72" rot="R180"/>
<instance part="GND10" gate="1" x="63.5" y="-12.7"/>
<instance part="IC2" gate="G$1" x="-109.22" y="-63.5" smashed="yes">
<attribute name="NAME" x="-102.87" y="-58.42" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-105.41" y="-73.66" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="IC3" gate="G$1" x="-109.22" y="-83.82" smashed="yes">
<attribute name="NAME" x="-102.87" y="-78.74" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-107.95" y="-93.98" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="GND2" gate="1" x="-127" y="-99.06"/>
<instance part="+3V33" gate="G$1" x="-68.58" y="-53.34"/>
<instance part="J2" gate="G$1" x="96.52" y="-12.7"/>
<instance part="C3" gate="G$1" x="-86.36" y="-30.48" smashed="yes" rot="R90">
<attribute name="NAME" x="-88.265" y="-28.575" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="-86.36" y="-33.9725" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="GND4" gate="1" x="-86.36" y="-38.1"/>
<instance part="GND25" gate="1" x="86.36" y="-20.32"/>
<instance part="GND26" gate="1" x="-58.42" y="76.2"/>
<instance part="R9" gate="G$1" x="-134.62" y="-25.4" smashed="yes" rot="R90">
<attribute name="NAME" x="-131.318" y="-27.2034" size="1.778" layer="95"/>
<attribute name="VALUE" x="-128.524" y="-22.86" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="C14" gate="G$1" x="55.88" y="-91.44" smashed="yes" rot="R180">
<attribute name="NAME" x="61.595" y="-85.725" size="1.778" layer="95" rot="R270"/>
<attribute name="VALUE" x="59.3725" y="-83.82" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="GND27" gate="1" x="10.16" y="-106.68"/>
</instances>
<busses>
</busses>
<nets>
<net name="GND" class="0">
<segment>
<pinref part="GND24" gate="1" pin="GND"/>
<wire x1="78.74" y1="-104.14" x2="78.74" y2="-109.22" width="0.1524" layer="91"/>
<pinref part="JP2" gate="G$1" pin="3"/>
</segment>
<segment>
<pinref part="GND7" gate="1" pin="GND"/>
<pinref part="C5" gate="G$1" pin="1"/>
<wire x1="-116.84" y1="68.58" x2="-116.84" y2="58.42" width="0.1524" layer="91"/>
<wire x1="-116.84" y1="58.42" x2="-109.22" y2="58.42" width="0.1524" layer="91"/>
<pinref part="C6" gate="G$1" pin="1"/>
<wire x1="-109.22" y1="58.42" x2="-109.22" y2="68.58" width="0.1524" layer="91"/>
<junction x="-109.22" y="58.42"/>
</segment>
<segment>
<pinref part="Y1" gate="G$1" pin="GND_2"/>
<pinref part="GND6" gate="1" pin="GND"/>
<wire x1="-127" y1="35.56" x2="-152.4" y2="35.56" width="0.1524" layer="91"/>
<wire x1="-152.4" y1="35.56" x2="-152.4" y2="30.48" width="0.1524" layer="91"/>
<pinref part="C4" gate="G$1" pin="2"/>
<wire x1="-132.08" y1="30.48" x2="-152.4" y2="30.48" width="0.1524" layer="91"/>
<junction x="-152.4" y="30.48"/>
</segment>
<segment>
<pinref part="Y1" gate="G$1" pin="GND_1"/>
<pinref part="GND8" gate="1" pin="GND"/>
<wire x1="-96.52" y1="33.02" x2="-96.52" y2="30.48" width="0.1524" layer="91"/>
<pinref part="C8" gate="G$1" pin="2"/>
<wire x1="-96.52" y1="30.48" x2="-96.52" y2="17.78" width="0.1524" layer="91"/>
<wire x1="-91.44" y1="30.48" x2="-96.52" y2="30.48" width="0.1524" layer="91"/>
<junction x="-96.52" y="30.48"/>
</segment>
<segment>
<pinref part="U1" gate="A" pin="GND_2"/>
<pinref part="GND12" gate="1" pin="GND"/>
<wire x1="-27.94" y1="35.56" x2="-50.8" y2="35.56" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND17" gate="1" pin="GND"/>
<pinref part="U1" gate="A" pin="GND"/>
<wire x1="127" y1="68.58" x2="116.84" y2="68.58" width="0.1524" layer="91"/>
<pinref part="C10" gate="G$1" pin="2"/>
<wire x1="116.84" y1="68.58" x2="109.22" y2="68.58" width="0.1524" layer="91"/>
<wire x1="116.84" y1="66.04" x2="116.84" y2="68.58" width="0.1524" layer="91"/>
<junction x="116.84" y="68.58"/>
</segment>
<segment>
<pinref part="GND15" gate="1" pin="GND"/>
<pinref part="U1" gate="A" pin="GND_3"/>
<wire x1="111.76" y1="17.78" x2="109.22" y2="17.78" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="U1" gate="A" pin="UGND"/>
<pinref part="GND13" gate="1" pin="GND"/>
<wire x1="-27.94" y1="60.96" x2="-38.1" y2="60.96" width="0.1524" layer="91"/>
<pinref part="C9" gate="G$1" pin="1"/>
<wire x1="-38.1" y1="60.96" x2="-43.18" y2="60.96" width="0.1524" layer="91"/>
<wire x1="-38.1" y1="58.42" x2="-38.1" y2="60.96" width="0.1524" layer="91"/>
<junction x="-38.1" y="60.96"/>
</segment>
<segment>
<pinref part="J1" gate="A" pin="GND1"/>
<pinref part="GND11" gate="1" pin="GND"/>
<wire x1="-66.04" y1="71.12" x2="-71.12" y2="71.12" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="C7" gate="G$1" pin="1"/>
<pinref part="GND9" gate="1" pin="GND"/>
<wire x1="-91.44" y1="58.42" x2="-91.44" y2="66.04" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="U1" gate="A" pin="PE2_(HWB!)"/>
<pinref part="GND16" gate="1" pin="GND"/>
<wire x1="109.22" y1="43.18" x2="114.3" y2="43.18" width="0.1524" layer="91"/>
<pinref part="U1" gate="A" pin="GND_4"/>
<wire x1="114.3" y1="43.18" x2="124.46" y2="43.18" width="0.1524" layer="91"/>
<wire x1="109.22" y1="48.26" x2="114.3" y2="48.26" width="0.1524" layer="91"/>
<wire x1="114.3" y1="48.26" x2="114.3" y2="43.18" width="0.1524" layer="91"/>
<junction x="114.3" y="43.18"/>
</segment>
<segment>
<pinref part="IC4" gate="G$1" pin="VSS"/>
<wire x1="-71.12" y1="-25.4" x2="-81.28" y2="-25.4" width="0.1524" layer="91"/>
<wire x1="-81.28" y1="-25.4" x2="-81.28" y2="-43.18" width="0.1524" layer="91"/>
<pinref part="GND5" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="IC1" gate="G$1" pin="GND"/>
<pinref part="GND1" gate="1" pin="GND"/>
<wire x1="-129.54" y1="-15.24" x2="-142.24" y2="-15.24" width="0.1524" layer="91"/>
<pinref part="C1" gate="G$1" pin="1"/>
<wire x1="-142.24" y1="-15.24" x2="-142.24" y2="-30.48" width="0.1524" layer="91"/>
<junction x="-142.24" y="-15.24"/>
<pinref part="R9" gate="G$1" pin="1"/>
<wire x1="-142.24" y1="-30.48" x2="-142.24" y2="-35.56" width="0.1524" layer="91"/>
<wire x1="-134.62" y1="-30.48" x2="-142.24" y2="-30.48" width="0.1524" layer="91"/>
<junction x="-142.24" y="-30.48"/>
</segment>
<segment>
<pinref part="C2" gate="G$1" pin="1"/>
<pinref part="GND3" gate="1" pin="GND"/>
<wire x1="-99.06" y1="-27.94" x2="-99.06" y2="-20.32" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="IC5" gate="G$1" pin="V-"/>
<wire x1="53.34" y1="-109.22" x2="53.34" y2="-124.46" width="0.1524" layer="91"/>
<pinref part="GND21" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="GND23" gate="1" pin="GND"/>
<wire x1="66.04" y1="-91.44" x2="58.42" y2="-91.44" width="0.1524" layer="91"/>
<pinref part="C14" gate="G$1" pin="1"/>
</segment>
<segment>
<pinref part="GND20" gate="1" pin="GND"/>
<pinref part="C13" gate="G$1" pin="1"/>
<wire x1="35.56" y1="-71.12" x2="35.56" y2="-68.58" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="SV1" gate="1" pin="6"/>
<wire x1="15.24" y1="-15.24" x2="22.86" y2="-15.24" width="0.1524" layer="91"/>
<label x="17.78" y="-15.24" size="1.778" layer="95"/>
<pinref part="GND14" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="IC2" gate="G$1" pin="GND"/>
<wire x1="-109.22" y1="-66.04" x2="-127" y2="-66.04" width="0.1524" layer="91"/>
<wire x1="-127" y1="-66.04" x2="-127" y2="-86.36" width="0.1524" layer="91"/>
<pinref part="GND2" gate="1" pin="GND"/>
<pinref part="IC3" gate="G$1" pin="GND"/>
<wire x1="-127" y1="-86.36" x2="-127" y2="-96.52" width="0.1524" layer="91"/>
<wire x1="-109.22" y1="-86.36" x2="-127" y2="-86.36" width="0.1524" layer="91"/>
<junction x="-127" y="-86.36"/>
</segment>
<segment>
<pinref part="LED1" gate="G$1" pin="K"/>
<pinref part="GND22" gate="1" pin="GND"/>
<wire x1="50.8" y1="-43.18" x2="60.96" y2="-43.18" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="J2" gate="G$1" pin="2"/>
<pinref part="GND10" gate="1" pin="GND"/>
<wire x1="86.36" y1="-10.16" x2="63.5" y2="-10.16" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND4" gate="1" pin="GND"/>
<pinref part="C3" gate="G$1" pin="1"/>
<wire x1="-86.36" y1="-35.56" x2="-86.36" y2="-33.02" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="J2" gate="G$1" pin="SHIELD"/>
<pinref part="GND25" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="GND19" gate="1" pin="GND"/>
<pinref part="S1" gate="G$1" pin="4"/>
<wire x1="114.3" y1="-45.72" x2="109.22" y2="-45.72" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="J1" gate="A" pin="CASE_GND"/>
<pinref part="GND26" gate="1" pin="GND"/>
<wire x1="-63.5" y1="78.74" x2="-58.42" y2="78.74" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="C12" gate="G$1" pin="1"/>
<pinref part="GND18" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="C11" gate="G$1" pin="1"/>
<pinref part="GND27" gate="1" pin="GND"/>
</segment>
</net>
<net name="+5V" class="0">
<segment>
<pinref part="P+2" gate="1" pin="+5V"/>
<wire x1="-109.22" y1="86.36" x2="-109.22" y2="78.74" width="0.1524" layer="91"/>
<wire x1="-109.22" y1="78.74" x2="-109.22" y2="73.66" width="0.1524" layer="91"/>
<junction x="-109.22" y="78.74"/>
<pinref part="C5" gate="G$1" pin="2"/>
<wire x1="-109.22" y1="78.74" x2="-116.84" y2="78.74" width="0.1524" layer="91"/>
<wire x1="-116.84" y1="78.74" x2="-116.84" y2="73.66" width="0.1524" layer="91"/>
<pinref part="C6" gate="G$1" pin="2"/>
<pinref part="F1" gate="G$1" pin="1"/>
</segment>
<segment>
<pinref part="P+10" gate="1" pin="+5V"/>
<wire x1="78.74" y1="-99.06" x2="78.74" y2="-93.98" width="0.1524" layer="91"/>
<pinref part="JP2" gate="G$1" pin="1"/>
</segment>
<segment>
<pinref part="U1" gate="A" pin="AVCC_2"/>
<pinref part="P+8" gate="1" pin="+5V"/>
<wire x1="109.22" y1="20.32" x2="124.46" y2="20.32" width="0.1524" layer="91"/>
<wire x1="124.46" y1="20.32" x2="124.46" y2="22.86" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="U1" gate="A" pin="VCC"/>
<wire x1="109.22" y1="45.72" x2="121.92" y2="45.72" width="0.1524" layer="91"/>
<wire x1="121.92" y1="45.72" x2="121.92" y2="50.8" width="0.1524" layer="91"/>
<pinref part="P+7" gate="1" pin="+5V"/>
</segment>
<segment>
<pinref part="U1" gate="A" pin="AVCC"/>
<pinref part="P+6" gate="1" pin="+5V"/>
<wire x1="109.22" y1="71.12" x2="111.76" y2="71.12" width="0.1524" layer="91"/>
<wire x1="111.76" y1="71.12" x2="111.76" y2="76.2" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="P+1" gate="1" pin="+5V"/>
<wire x1="-142.24" y1="-10.16" x2="-142.24" y2="2.54" width="0.1524" layer="91"/>
<pinref part="C1" gate="G$1" pin="2"/>
<pinref part="IC1" gate="G$1" pin="VIN"/>
<wire x1="-129.54" y1="-12.7" x2="-129.54" y2="-10.16" width="0.1524" layer="91"/>
<wire x1="-129.54" y1="-10.16" x2="-142.24" y2="-10.16" width="0.1524" layer="91"/>
<junction x="-142.24" y="-10.16"/>
</segment>
<segment>
<pinref part="P+9" gate="1" pin="+5V"/>
<wire x1="53.34" y1="-93.98" x2="53.34" y2="-91.44" width="0.1524" layer="91"/>
<pinref part="IC5" gate="G$1" pin="V+"/>
<pinref part="C14" gate="G$1" pin="2"/>
<wire x1="53.34" y1="-91.44" x2="53.34" y2="-83.82" width="0.1524" layer="91"/>
<junction x="53.34" y="-91.44"/>
</segment>
<segment>
<pinref part="P+4" gate="1" pin="+5V"/>
<pinref part="R4" gate="G$1" pin="2"/>
<wire x1="-48.26" y1="40.64" x2="-50.8" y2="40.64" width="0.1524" layer="91"/>
<pinref part="U1" gate="A" pin="VCC_2"/>
<wire x1="-50.8" y1="38.1" x2="-27.94" y2="38.1" width="0.1524" layer="91"/>
<wire x1="-50.8" y1="38.1" x2="-50.8" y2="40.64" width="0.1524" layer="91"/>
<junction x="-50.8" y="40.64"/>
</segment>
</net>
<net name="N$5" class="0">
<segment>
<pinref part="R7" gate="G$1" pin="1"/>
<wire x1="27.94" y1="-63.5" x2="35.56" y2="-63.5" width="0.1524" layer="91"/>
<junction x="27.94" y="-63.5"/>
<pinref part="JP1" gate="G$1" pin="1"/>
<wire x1="45.72" y1="-63.5" x2="35.56" y2="-63.5" width="0.1524" layer="91"/>
<pinref part="C13" gate="G$1" pin="2"/>
<junction x="35.56" y="-63.5"/>
</segment>
</net>
<net name="XTAL1" class="0">
<segment>
<pinref part="U1" gate="A" pin="XTAL1"/>
<wire x1="-27.94" y1="30.48" x2="-35.56" y2="30.48" width="0.1524" layer="91"/>
<label x="-35.56" y="30.48" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="Y1" gate="G$1" pin="1"/>
<pinref part="C4" gate="G$1" pin="1"/>
<wire x1="-127" y1="33.02" x2="-127" y2="30.48" width="0.1524" layer="91"/>
<wire x1="-127" y1="30.48" x2="-127" y2="22.86" width="0.1524" layer="91"/>
<wire x1="-127" y1="22.86" x2="-116.84" y2="22.86" width="0.1524" layer="91"/>
<label x="-119.38" y="22.86" size="1.778" layer="95"/>
<junction x="-127" y="30.48"/>
</segment>
</net>
<net name="XTAL2" class="0">
<segment>
<pinref part="U1" gate="A" pin="XTAL2"/>
<wire x1="-27.94" y1="33.02" x2="-35.56" y2="33.02" width="0.1524" layer="91"/>
<label x="-35.56" y="33.02" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="Y1" gate="G$1" pin="3"/>
<pinref part="C8" gate="G$1" pin="1"/>
<wire x1="-96.52" y1="35.56" x2="-91.44" y2="35.56" width="0.1524" layer="91"/>
<wire x1="-91.44" y1="35.56" x2="-83.82" y2="35.56" width="0.1524" layer="91"/>
<label x="-86.36" y="35.56" size="1.778" layer="95"/>
<junction x="-91.44" y="35.56"/>
</segment>
</net>
<net name="VCC" class="0">
<segment>
<pinref part="U1" gate="A" pin="VBUS"/>
<wire x1="-27.94" y1="55.88" x2="-45.72" y2="55.88" width="0.1524" layer="91"/>
<pinref part="U1" gate="A" pin="UVCC"/>
<wire x1="-27.94" y1="68.58" x2="-45.72" y2="68.58" width="0.1524" layer="91"/>
<wire x1="-45.72" y1="68.58" x2="-45.72" y2="81.28" width="0.1524" layer="91"/>
<wire x1="-45.72" y1="55.88" x2="-45.72" y2="68.58" width="0.1524" layer="91"/>
<junction x="-45.72" y="68.58"/>
<pinref part="P+5" gate="VCC" pin="VCC"/>
</segment>
<segment>
<pinref part="P+3" gate="VCC" pin="VCC"/>
<pinref part="C7" gate="G$1" pin="2"/>
<wire x1="-91.44" y1="71.12" x2="-91.44" y2="78.74" width="0.1524" layer="91"/>
<pinref part="J1" gate="A" pin="VBUS1"/>
<wire x1="-91.44" y1="78.74" x2="-91.44" y2="86.36" width="0.1524" layer="91"/>
<wire x1="-81.28" y1="71.12" x2="-91.44" y2="71.12" width="0.1524" layer="91"/>
<junction x="-91.44" y="71.12"/>
<pinref part="F1" gate="G$1" pin="2"/>
<junction x="-91.44" y="78.74"/>
</segment>
</net>
<net name="N$2" class="0">
<segment>
<pinref part="U1" gate="A" pin="UCAP"/>
<pinref part="C9" gate="G$1" pin="2"/>
<wire x1="-27.94" y1="58.42" x2="-33.02" y2="58.42" width="0.1524" layer="91"/>
</segment>
</net>
<net name="D_N" class="0">
<segment>
<pinref part="U1" gate="A" pin="D-"/>
<wire x1="-27.94" y1="66.04" x2="-30.48" y2="66.04" width="0.1524" layer="91"/>
<label x="-30.48" y="66.04" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="R2" gate="G$1" pin="2"/>
<wire x1="-78.74" y1="58.42" x2="-78.74" y2="55.88" width="0.1524" layer="91"/>
<wire x1="-78.74" y1="55.88" x2="-66.04" y2="55.88" width="0.1524" layer="91"/>
<label x="-68.58" y="55.88" size="1.778" layer="95"/>
</segment>
</net>
<net name="D_P" class="0">
<segment>
<pinref part="U1" gate="A" pin="D+"/>
<wire x1="-27.94" y1="63.5" x2="-30.48" y2="63.5" width="0.1524" layer="91"/>
<label x="-30.48" y="63.5" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="R3" gate="G$1" pin="2"/>
<wire x1="-76.2" y1="58.42" x2="-66.04" y2="58.42" width="0.1524" layer="91"/>
<label x="-68.58" y="58.42" size="1.778" layer="95"/>
</segment>
</net>
<net name="LED_IND" class="0">
<segment>
<pinref part="R8" gate="G$1" pin="2"/>
<wire x1="22.86" y1="-43.18" x2="12.7" y2="-43.18" width="0.1524" layer="91"/>
<label x="12.7" y="-43.18" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="U1" gate="A" pin="PD7_(T0/OC4D/ADC10)"/>
<wire x1="109.22" y1="27.94" x2="119.38" y2="27.94" width="0.1524" layer="91"/>
<label x="109.22" y="27.94" size="1.778" layer="95"/>
</segment>
</net>
<net name="SCLK" class="0">
<segment>
<pinref part="U1" gate="A" pin="(PCINT1/SCLK)_PB1"/>
<wire x1="-27.94" y1="50.8" x2="-33.02" y2="50.8" width="0.1524" layer="91"/>
<label x="-33.02" y="50.8" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="SV1" gate="1" pin="3"/>
<wire x1="0" y1="-12.7" x2="-7.62" y2="-12.7" width="0.1524" layer="91"/>
<label x="-7.62" y="-12.7" size="1.778" layer="95"/>
</segment>
<segment>
<label x="-116.84" y="-68.58" size="1.778" layer="95"/>
<wire x1="-109.22" y1="-68.58" x2="-119.38" y2="-68.58" width="0.1524" layer="91"/>
<pinref part="IC2" gate="G$1" pin="2A"/>
</segment>
</net>
<net name="MOSI" class="0">
<segment>
<pinref part="U1" gate="A" pin="(PDI/PCINT2/MOSI)_PB2"/>
<wire x1="-27.94" y1="48.26" x2="-33.02" y2="48.26" width="0.1524" layer="91"/>
<label x="-33.02" y="48.26" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="SV1" gate="1" pin="4"/>
<wire x1="15.24" y1="-12.7" x2="22.86" y2="-12.7" width="0.1524" layer="91"/>
<label x="17.78" y="-12.7" size="1.778" layer="95"/>
</segment>
<segment>
<label x="-116.84" y="-63.5" size="1.778" layer="95"/>
<wire x1="-109.22" y1="-63.5" x2="-116.84" y2="-63.5" width="0.1524" layer="91"/>
<pinref part="IC2" gate="G$1" pin="1A"/>
</segment>
</net>
<net name="MISO" class="0">
<segment>
<pinref part="U1" gate="A" pin="(PDO/PCINT3/MISO)_PB3"/>
<wire x1="-27.94" y1="45.72" x2="-33.02" y2="45.72" width="0.1524" layer="91"/>
<label x="-33.02" y="45.72" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="SV1" gate="1" pin="1"/>
<wire x1="0" y1="-10.16" x2="-7.62" y2="-10.16" width="0.1524" layer="91"/>
<label x="-7.62" y="-10.16" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="IC3" gate="G$1" pin="1Y"/>
<wire x1="-83.82" y1="-83.82" x2="-71.12" y2="-83.82" width="0.1524" layer="91"/>
<label x="-83.82" y="-83.82" size="1.778" layer="95"/>
</segment>
</net>
<net name="/RESET" class="0">
<segment>
<pinref part="U1" gate="A" pin="!"/>
<wire x1="-27.94" y1="40.64" x2="-38.1" y2="40.64" width="0.1524" layer="91"/>
<label x="-33.02" y="40.64" size="1.778" layer="95"/>
<pinref part="R4" gate="G$1" pin="1"/>
</segment>
<segment>
<pinref part="SV1" gate="1" pin="5"/>
<wire x1="0" y1="-15.24" x2="-7.62" y2="-15.24" width="0.1524" layer="91"/>
<label x="-7.62" y="-15.24" size="1.778" layer="95"/>
</segment>
</net>
<net name="D1_N" class="0">
<segment>
<pinref part="J1" gate="A" pin="D+1"/>
<wire x1="-76.2" y1="71.12" x2="-76.2" y2="68.58" width="0.1524" layer="91"/>
<pinref part="R3" gate="G$1" pin="1"/>
</segment>
</net>
<net name="D1_P" class="0">
<segment>
<pinref part="J1" gate="A" pin="D-1"/>
<wire x1="-78.74" y1="71.12" x2="-78.74" y2="68.58" width="0.1524" layer="91"/>
<pinref part="R2" gate="G$1" pin="1"/>
</segment>
</net>
<net name="SCL" class="0">
<segment>
<pinref part="U1" gate="A" pin="(OC0B/SCL/INT0)_PD0"/>
<label x="-35.56" y="27.94" size="1.778" layer="95"/>
<wire x1="-27.94" y1="27.94" x2="-35.56" y2="27.94" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="J2" gate="G$1" pin="3"/>
<wire x1="86.36" y1="-12.7" x2="76.2" y2="-12.7" width="0.1524" layer="91"/>
<label x="76.2" y="-12.7" size="1.778" layer="95"/>
</segment>
</net>
<net name="SDA" class="0">
<segment>
<pinref part="U1" gate="A" pin="(SDA/INT1)_PD1"/>
<label x="-35.56" y="25.4" size="1.778" layer="95"/>
<wire x1="-27.94" y1="25.4" x2="-35.56" y2="25.4" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="J2" gate="G$1" pin="1"/>
<wire x1="86.36" y1="-7.62" x2="76.2" y2="-7.62" width="0.1524" layer="91"/>
<label x="76.2" y="-7.62" size="1.778" layer="95"/>
</segment>
</net>
<net name="RXD1" class="0">
<segment>
<pinref part="U1" gate="A" pin="(RXD1/INT2)_PD2"/>
<wire x1="-27.94" y1="22.86" x2="-35.56" y2="22.86" width="0.1524" layer="91"/>
<label x="-35.56" y="22.86" size="1.778" layer="95"/>
</segment>
</net>
<net name="TXD1" class="0">
<segment>
<pinref part="U1" gate="A" pin="(TXD1/INT3)_PD3"/>
<wire x1="-27.94" y1="20.32" x2="-35.56" y2="20.32" width="0.1524" layer="91"/>
<label x="-35.56" y="20.32" size="1.778" layer="95"/>
</segment>
</net>
<net name="N$4" class="0">
<segment>
<pinref part="U1" gate="A" pin="AREF"/>
<pinref part="C10" gate="G$1" pin="1"/>
<wire x1="109.22" y1="66.04" x2="111.76" y2="66.04" width="0.1524" layer="91"/>
</segment>
</net>
<net name="PWM_OUT" class="0">
<segment>
<pinref part="U1" gate="A" pin="PB5_(PCINT5/OC1A/OC4B/ADC12)"/>
<wire x1="109.22" y1="33.02" x2="116.84" y2="33.02" width="0.1524" layer="91"/>
<label x="109.22" y="33.02" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="R5" gate="G$1" pin="2"/>
<wire x1="-2.54" y1="-99.06" x2="-10.16" y2="-99.06" width="0.1524" layer="91"/>
<label x="-10.16" y="-99.06" size="1.778" layer="95"/>
</segment>
</net>
<net name="PWM_LED" class="0">
<segment>
<pinref part="U1" gate="A" pin="PB6_(PCINT6/OC1B/OC4B/ADC13)"/>
<wire x1="109.22" y1="35.56" x2="116.84" y2="35.56" width="0.1524" layer="91"/>
<label x="109.22" y="35.56" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="R7" gate="G$1" pin="2"/>
<wire x1="17.78" y1="-63.5" x2="12.7" y2="-63.5" width="0.1524" layer="91"/>
<label x="12.7" y="-63.5" size="1.778" layer="95"/>
</segment>
</net>
<net name="LEDSW2" class="0">
<segment>
<pinref part="PAD1" gate="G$1" pin="P"/>
<wire x1="43.18" y1="-83.82" x2="12.7" y2="-83.82" width="0.1524" layer="91"/>
<label x="12.7" y="-83.82" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="U1" gate="A" pin="(XCK1/CTS!)_PD5"/>
<wire x1="-27.94" y1="17.78" x2="-35.56" y2="17.78" width="0.1524" layer="91"/>
<label x="-35.56" y="17.78" size="1.778" layer="95"/>
</segment>
</net>
<net name="LEDSW1" class="0">
<segment>
<pinref part="U1" gate="A" pin="PB4_(PCINT4/ADC11)"/>
<wire x1="109.22" y1="30.48" x2="116.84" y2="30.48" width="0.1524" layer="91"/>
<label x="109.22" y="30.48" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="JP1" gate="G$1" pin="2"/>
<wire x1="45.72" y1="-66.04" x2="45.72" y2="-78.74" width="0.1524" layer="91"/>
<wire x1="45.72" y1="-78.74" x2="12.7" y2="-78.74" width="0.1524" layer="91"/>
<label x="12.7" y="-78.74" size="1.778" layer="95"/>
</segment>
</net>
<net name="PB" class="0">
<segment>
<pinref part="U1" gate="A" pin="PC6_(OC3A/OC4A!)"/>
<wire x1="109.22" y1="38.1" x2="116.84" y2="38.1" width="0.1524" layer="91"/>
<label x="109.22" y="38.1" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="S1" gate="G$1" pin="1"/>
<wire x1="109.22" y1="-43.18" x2="121.92" y2="-43.18" width="0.1524" layer="91"/>
<label x="111.76" y="-43.18" size="1.778" layer="95"/>
</segment>
</net>
<net name="SCLK1" class="0">
<segment>
<pinref part="IC4" gate="G$1" pin="SCLK"/>
<wire x1="-71.12" y1="-17.78" x2="-78.74" y2="-17.78" width="0.1524" layer="91"/>
<label x="-78.74" y="-17.78" size="1.778" layer="95"/>
</segment>
<segment>
<label x="-83.82" y="-68.58" size="1.778" layer="95"/>
<wire x1="-83.82" y1="-68.58" x2="-73.66" y2="-68.58" width="0.1524" layer="91"/>
<pinref part="IC2" gate="G$1" pin="2Y"/>
</segment>
</net>
<net name="MISO1" class="0">
<segment>
<pinref part="IC4" gate="G$1" pin="MISO"/>
<wire x1="-71.12" y1="-20.32" x2="-78.74" y2="-20.32" width="0.1524" layer="91"/>
<label x="-78.74" y="-20.32" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="IC3" gate="G$1" pin="1A"/>
<wire x1="-109.22" y1="-83.82" x2="-124.46" y2="-83.82" width="0.1524" layer="91"/>
<label x="-124.46" y="-83.82" size="1.778" layer="95"/>
</segment>
</net>
<net name="MOSI1" class="0">
<segment>
<pinref part="IC4" gate="G$1" pin="MOSI"/>
<wire x1="-71.12" y1="-22.86" x2="-78.74" y2="-22.86" width="0.1524" layer="91"/>
<label x="-78.74" y="-22.86" size="1.778" layer="95"/>
</segment>
<segment>
<label x="-83.82" y="-63.5" size="1.778" layer="95"/>
<wire x1="-83.82" y1="-63.5" x2="-73.66" y2="-63.5" width="0.1524" layer="91"/>
<pinref part="IC2" gate="G$1" pin="1Y"/>
</segment>
</net>
<net name="FRAME1" class="0">
<segment>
<pinref part="IC4" gate="G$1" pin="FRAME"/>
<wire x1="-71.12" y1="-33.02" x2="-78.74" y2="-33.02" width="0.1524" layer="91"/>
<label x="-78.74" y="-33.02" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="R1" gate="G$1" pin="2"/>
<wire x1="-106.68" y1="-106.68" x2="-111.76" y2="-106.68" width="0.1524" layer="91"/>
<label x="-111.76" y="-106.68" size="1.778" layer="95"/>
</segment>
</net>
<net name="CS1" class="0">
<segment>
<pinref part="IC4" gate="G$1" pin="CS"/>
<wire x1="-71.12" y1="-35.56" x2="-78.74" y2="-35.56" width="0.1524" layer="91"/>
<label x="-78.74" y="-35.56" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="IC3" gate="G$1" pin="2Y"/>
<wire x1="-83.82" y1="-88.9" x2="-71.12" y2="-88.9" width="0.1524" layer="91"/>
<label x="-83.82" y="-88.9" size="1.778" layer="95"/>
</segment>
</net>
<net name="+3V3" class="0">
<segment>
<pinref part="IC4" gate="G$1" pin="V3V3A"/>
<pinref part="IC4" gate="G$1" pin="V3V3B"/>
<wire x1="-71.12" y1="-30.48" x2="-71.12" y2="-27.94" width="0.1524" layer="91"/>
<wire x1="-71.12" y1="-27.94" x2="-86.36" y2="-27.94" width="0.1524" layer="91"/>
<pinref part="+3V32" gate="G$1" pin="+3V3"/>
<wire x1="-86.36" y1="-27.94" x2="-86.36" y2="-15.24" width="0.1524" layer="91"/>
<junction x="-71.12" y="-27.94"/>
<pinref part="C3" gate="G$1" pin="2"/>
<junction x="-86.36" y="-27.94"/>
</segment>
<segment>
<pinref part="IC1" gate="G$1" pin="VOUT"/>
<wire x1="-101.6" y1="-15.24" x2="-99.06" y2="-15.24" width="0.1524" layer="91"/>
<wire x1="-99.06" y1="-15.24" x2="-99.06" y2="0" width="0.1524" layer="91"/>
<pinref part="+3V31" gate="G$1" pin="+3V3"/>
<pinref part="C2" gate="G$1" pin="2"/>
<junction x="-99.06" y="-15.24"/>
</segment>
<segment>
<pinref part="+3V33" gate="G$1" pin="+3V3"/>
<pinref part="IC3" gate="G$1" pin="VCC"/>
<wire x1="-83.82" y1="-86.36" x2="-68.58" y2="-86.36" width="0.1524" layer="91"/>
<wire x1="-68.58" y1="-86.36" x2="-68.58" y2="-66.04" width="0.1524" layer="91"/>
<pinref part="IC2" gate="G$1" pin="VCC"/>
<wire x1="-68.58" y1="-66.04" x2="-68.58" y2="-55.88" width="0.1524" layer="91"/>
<wire x1="-83.82" y1="-66.04" x2="-68.58" y2="-66.04" width="0.1524" layer="91"/>
<junction x="-68.58" y="-66.04"/>
</segment>
</net>
<net name="N$17" class="0">
<segment>
<pinref part="LED1" gate="G$1" pin="A"/>
<pinref part="R8" gate="G$1" pin="1"/>
<wire x1="35.56" y1="-43.18" x2="33.02" y2="-43.18" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$6" class="0">
<segment>
<pinref part="R5" gate="G$1" pin="1"/>
<pinref part="R6" gate="G$1" pin="2"/>
<wire x1="7.62" y1="-99.06" x2="10.16" y2="-99.06" width="0.1524" layer="91"/>
<pinref part="C11" gate="G$1" pin="2"/>
<wire x1="10.16" y1="-99.06" x2="15.24" y2="-99.06" width="0.1524" layer="91"/>
<junction x="10.16" y="-99.06"/>
</segment>
</net>
<net name="N$7" class="0">
<segment>
<pinref part="R6" gate="G$1" pin="1"/>
<pinref part="IC5" gate="G$1" pin="+IN"/>
<wire x1="25.4" y1="-99.06" x2="27.94" y2="-99.06" width="0.1524" layer="91"/>
<pinref part="C12" gate="G$1" pin="2"/>
<wire x1="27.94" y1="-99.06" x2="45.72" y2="-99.06" width="0.1524" layer="91"/>
<junction x="27.94" y="-99.06"/>
</segment>
</net>
<net name="CS" class="0">
<segment>
<pinref part="U1" gate="A" pin="INT.6/AIN0)PE6"/>
<wire x1="-27.94" y1="71.12" x2="-33.02" y2="71.12" width="0.1524" layer="91"/>
<label x="-33.02" y="71.12" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="IC3" gate="G$1" pin="2A"/>
<wire x1="-109.22" y1="-88.9" x2="-124.46" y2="-88.9" width="0.1524" layer="91"/>
<label x="-124.46" y="-88.9" size="1.778" layer="95"/>
</segment>
</net>
<net name="FRAME" class="0">
<segment>
<pinref part="R1" gate="G$1" pin="1"/>
<wire x1="-96.52" y1="-106.68" x2="-83.82" y2="-106.68" width="0.1524" layer="91"/>
<label x="-88.9" y="-106.68" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="U1" gate="A" pin="PF4_(ADC4/TCK)"/>
<wire x1="109.22" y1="58.42" x2="116.84" y2="58.42" width="0.1524" layer="91"/>
<label x="109.22" y="58.42" size="1.778" layer="95"/>
</segment>
</net>
<net name="ADC9" class="0">
<segment>
<pinref part="IC5" gate="G$1" pin="OUT"/>
<wire x1="60.96" y1="-101.6" x2="60.96" y2="-114.3" width="0.1524" layer="91"/>
<wire x1="60.96" y1="-114.3" x2="45.72" y2="-114.3" width="0.1524" layer="91"/>
<pinref part="IC5" gate="G$1" pin="-IN"/>
<wire x1="45.72" y1="-114.3" x2="45.72" y2="-104.14" width="0.1524" layer="91"/>
<wire x1="60.96" y1="-101.6" x2="78.74" y2="-101.6" width="0.1524" layer="91"/>
<pinref part="JP2" gate="G$1" pin="2"/>
<junction x="60.96" y="-101.6"/>
</segment>
</net>
<net name="POW_EN" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="EN"/>
<wire x1="-129.54" y1="-17.78" x2="-134.62" y2="-17.78" width="0.1524" layer="91"/>
<label x="-139.7" y="-20.32" size="1.778" layer="95"/>
<pinref part="R9" gate="G$1" pin="2"/>
<wire x1="-134.62" y1="-17.78" x2="-137.16" y2="-17.78" width="0.1524" layer="91"/>
<wire x1="-134.62" y1="-17.78" x2="-134.62" y2="-20.32" width="0.1524" layer="91"/>
<junction x="-134.62" y="-17.78"/>
</segment>
<segment>
<pinref part="SV1" gate="1" pin="2"/>
<wire x1="15.24" y1="-10.16" x2="22.86" y2="-10.16" width="0.1524" layer="91"/>
<label x="17.78" y="-10.16" size="1.778" layer="95"/>
<label x="17.78" y="-10.16" size="1.778" layer="95"/>
<label x="17.78" y="-10.16" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="U1" gate="A" pin="(SS/PCINT0)_PB0"/>
<wire x1="-27.94" y1="53.34" x2="-38.1" y2="53.34" width="0.1524" layer="91"/>
<label x="-38.1" y="53.34" size="1.778" layer="95"/>
</segment>
</net>
<net name="PF5" class="0">
<segment>
<pinref part="U1" gate="A" pin="PF5_(ADC5/TMS)"/>
<wire x1="109.22" y1="55.88" x2="116.84" y2="55.88" width="0.1524" layer="91"/>
<label x="109.22" y="55.88" size="1.778" layer="95"/>
</segment>
</net>
<net name="PF6" class="0">
<segment>
<pinref part="U1" gate="A" pin="PF6_(ADC6/TDO)"/>
<wire x1="109.22" y1="53.34" x2="116.84" y2="53.34" width="0.1524" layer="91"/>
<label x="109.22" y="53.34" size="1.778" layer="95"/>
</segment>
</net>
<net name="PF7" class="0">
<segment>
<pinref part="U1" gate="A" pin="PF7_(ADC7/TDI)"/>
<wire x1="109.22" y1="50.8" x2="116.84" y2="50.8" width="0.1524" layer="91"/>
<label x="109.22" y="50.8" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD4" class="0">
<segment>
<pinref part="U1" gate="A" pin="PD4_(ICP1/ADC8"/>
<wire x1="109.22" y1="22.86" x2="114.3" y2="22.86" width="0.1524" layer="91"/>
<label x="109.22" y="22.86" size="1.778" layer="95"/>
</segment>
</net>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
<compatibility>
<note version="6.3" minversion="6.2.2" severity="warning">
Since Version 6.2.2 text objects can contain more than one line,
which will not be processed correctly with this version.
</note>
</compatibility>
</eagle>
