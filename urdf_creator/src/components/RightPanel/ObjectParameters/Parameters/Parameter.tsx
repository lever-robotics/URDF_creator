import { InputHTMLAttributes, ReactNode } from "react";

type Props = React.DetailedHTMLProps<InputHTMLAttributes<HTMLInputElement>, HTMLInputElement> & {units?: ReactNode}

export default function Parameter(props: Props) {
    return (
        <li>
            <strong>{props.title}</strong>
            <input {...props} />
            <span className="units">{props.units}</span>
        </li>
    );
}
