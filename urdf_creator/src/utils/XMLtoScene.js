// class unknownTag {
//     constructor(type) {
//         this.type = type;
//     }
// }

// class Link {
//     constructor(name) {
//         this.name = name;
//     }
// }

// class Joint {
//     constructor(name, type) {
//         this.name = name;
//         this.type = type;
//     }
// }

// class Parent {
//     constructor(linkName) {
//         this.linkName = linkName;
//     }
// }

// class Child {
//     constructor(linkName) {
//         this.linkName = linkName;
//     }
// }

// class Origin {
//     constructor(xyz, rpy) {
//         this.xyz = xyz;
//         this.rpy = rpy;
//     }
// }

// class Space {}

// class Main {
//     run() {
//         this.parseXml(`

//               <link name="base_footprint"/>

//               <joint name="base_joint" type="fixed">
//                 <parent link="base_footprint"/>
//                 <child link="base_link" />
//                 <origin xyz="0 0 0.010" rpy="0 0 0"/>
//               </joint>

//               <link name="base_link">
//                 <visual>
//                   <origin xyz="-0.064 0 0.0" rpy="0 0 0"/>
//                   <geometry>
//                     <mesh filename="package://turtlebot3_description/meshes/bases/waffle_base.stl" scale="0.001 0.001 0.001"/>
//                   </geometry>
//                   <material name="light_black"/>
//                 </visual>`);
//     }

//     extractData(line) {
//         const tagStart = line.indexOf("<");

//         if (tagStart === -1) {
//             return " ";
//         }

//         // finds the first space after the tag
//         let firstSpace = line.indexOf(" ", tagStart);

//         //if there is no space, look instead for the closing tag
//         if (firstSpace === -1) {
//             firstSpace = line.indexOf(">", tagStart);
//         }

//         //grabs the text between first tag and first space
//         const tagType = line.substring(tagStart + 1, firstSpace);

//         const data = {};
//         // go over all the values in the line and parse them into data
//         while (true) {
//             const equals = line.indexOf("=", firstSpace);
//             // when there are no more = in the line break out
//             if (equals === -1) break;

//             const dataName = line.substring(firstSpace + 1, equals);
//             // start searching for the second quote after the equals and the first quote
//             const endingQuote = line.indexOf('"', equals + 2);
//             const dataValue = line.substring(equals + 2, endingQuote);
//             data[dataName] = dataValue;
//             // update first space to be after the second quote
//             firstSpace = endingQuote + 1;
//         }

//         return [tagType, data];
//     }

//     findClosingTagLineIndex(lines, tagTypes, index) {
//         const query = tagTypes[index];

//         // start at the next line
//         index++;
//         // run until closing tag is found
//         while (index < lines.length) {
//             const currentLine = tagTypes[index];
//             if (currentLine.endsWith(query)) return index;
//             index++;
//         }
//         return -1;
//     }

//     parseXml(xml) {
//         const parsedLines = [];
//         const lines = xml.split("\n");
//         let index = 0;

//         const tagTypes = [];
//         const parsedData = [];

//         // get the tag types for each line of the file
//         for (const line of lines) {
//             const [tagType, data] = this.extractData(line);
//             tagTypes.push(tagType);
//             parsedData.push(data);
//         }

//         // process each tag
//         while (index < lines.length) {
//             const line = lines[index];
//             const tagType = tagTypes[index];

//             const isMultiLineTag = line.indexOf("/>") === -1;
//             let closingTagLineIndex;
//             if (isMultiLineTag) {
//                 closingTagLineIndex = this.findClosingTagLineIndex(lines, tagTypes, index);
//             }

//             const data = parsedData[index];

//             switch (tagType) {
//                 case "joint":
//                     const joint = new Joint(data.name, data.type);
//                     parsedLines.push(joint);
//                     index = closingTagLineIndex;
//                     break;
//                 case "link":
//                     parsedLines.push(new Link(line, "default name"));
//                     break;
//                 default:
//                     parsedLines.push(new unknownTag(tagType, line));
//                     break;
//             }
//             index++;
//         }
//         for (const obj of parsedLines) {
//             console./log(obj);
//         }
//         console./log("number of lines: ", lines.length);
//     }
// }

// new Main().run();
