import { InputHTMLAttributes, ReactNode } from "react";
import styles from "./Parameter.module.css";

type Props = React.DetailedHTMLProps<InputHTMLAttributes<HTMLInputElement>, HTMLInputElement> & {units?: ReactNode}

export default function Parameter(props: Props) {
    return (
        <div className={styles.parameter}>
            <strong className={styles.paramLabel}>{props.title}</strong>
            <input {...props} className={styles.input}/>
            {/* <span className={styles.units}>{props.units}</span> */}
        </div>
    );
}
