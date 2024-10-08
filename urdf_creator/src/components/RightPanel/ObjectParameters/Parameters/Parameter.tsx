import { ChangeEventHandler, InputHTMLAttributes, ReactNode } from "react";
import styles from "./Parameter.module.css";

type Props = React.DetailedHTMLProps<
    InputHTMLAttributes<HTMLInputElement>,
    HTMLInputElement
> & { units?: ReactNode; options?: {value:string, option:string}[], onSelectChange?: ChangeEventHandler<HTMLSelectElement>
    | undefined };

export default function Parameter(props: Props) {
    // <select
    //                         value={selectedObject.jointType}
    //                         onChange={handleJointTypeChange}
    //                         className={styles.select}>
    //                         <option value="fixed">Fixed</option>
    //                         <option value="revolute">Revolute</option>
    //                         <option value="continuous">Continuous</option>
    //                         <option value="prismatic">Prismatic</option>
    //                         {/* <option value="planar">Planar</option>
    //                     <option value="floating">Floating</option> */}
    //                     </select>

    if (props.type === "select") {
        return (
            <div className={styles.parameter}>
                <strong className={styles.paramLabel}>{props.title}</strong>
                <select
                    value={props.value}
                    onChange={props.onSelectChange}
                    className={props.className ?? styles.select}>
                    {props.options?.map((option, index) => (
                        <option key={index} value={option.value}>{option.option}</option>
                    ))}
                </select>
            </div>
        );
    } else {
        return (
            <div className={styles.parameter}>
                <strong className={styles.paramLabel}>{props.title}</strong>
                <input {...props} className={props.className ?? styles.input} />
                {/* <span className={styles.units}>{props.units}</span> */}
            </div>
        );
    }
}
